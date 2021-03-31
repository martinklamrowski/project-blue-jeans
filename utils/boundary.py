"""
Class to handle direct communication with CoppeliaSim using the RemoteAPI.
  - Contains some low-level getters and setters for CoppeliaSim scene object
    attributes.
  - Also contains some higher-level functions to perform an entire routine,
    like step the Robo forward one block.
"""
import math

import sim_lib.sim as sim
import sim_lib.simConst as sC

import utils.vec as vec


class Boundary(object):
    def __init__(self, port):
        # connect to CoppeliaSim
        sim.simxFinish(-1)  # just in case, close all opened connections
        self.__port = port
        self.__clientID = sim.simxStart("127.0.0.1", port, True, True, 5000, 0)
        if self.__clientID == -1:
            raise ValueError("""
                Sorry, issue connecting to CoppeliaSim.\n
                sim.simxStart("127.0.0.1", <{}>, True, True, 5000, 0) == -1
            """.format(port))

        # init robot parts
        self.__scene_objects, self.__proxies = self.__get_scene_objects_dict()

    def snap_to_angular_point(self, velocity, angular_point):
        """
        Function that handles all of the details relating to snapping the Robo's front
        to the specified angular point.

        :param velocity: float -> velocity to turn at; will actually turn at twice this value.
        :param angular_point: float -> angular point, probably as defined in constants.py.
        :return: None
        """

        object_name = "body"
        oriented = False

        # don't turn if we're already snapped to the angular point!
        starting_orientation = vec.euler_g_to_rad(self.get_orientation(object_name)[2])
        if math.fabs(vec.euler_g_to_rad(starting_orientation) - angular_point) < 0.05:
            return

        # start turning
        self.set_left_motor_velocity(velocity)
        self.set_right_motor_velocity(-velocity)

        while not oriented:
            euler_angles = self.get_orientation(object_name)
            g = euler_angles[2]

            if math.fabs(vec.euler_g_to_rad(g) - angular_point) < 0.05:
                oriented = True

        # stoohpe
        self.set_left_motor_velocity(0)
        self.set_right_motor_velocity(0)

    def step_forward(self, velocity, angular_point):
        """
        Handles all of the details for moving forward a specified number
        of steps (blocks).

        :param velocity: float -> velocity to move forward at.
        :param angular_point: float -> the heading.
        :return: None
        """
        object_name = "body"

        # need to know how the coordinates will be updating; True for increase
        moving_in_x = None
        moving_in_y = None
        there = False

        if angular_point == 0 or angular_point == math.pi:
            # north or south
            moving_in_y = True if angular_point == 0 else False
            print("north or south")
        elif angular_point == 3 * math.pi / 2 or angular_point == math.pi / 2:
            # east or west
            moving_in_x = True if angular_point == 3 * math.pi / 2 else False
            print("east or west")

        # GPS baby!
        # but this actually isn't that bad; if need be, presumably
        # the 'getPosition' method could be redone by some big brain
        # who has time and money to implement some SpaceX level gyroscopes
        # and stuff to get the orientation and location of the robot -
        # yay for decoupled code!
        pos_start = self.get_position(object_name)
        if moving_in_x is not None:
            pos_start = pos_start[0]  # working with x
        elif moving_in_y is not None:
            pos_start = pos_start[1]  # working with y

        # this is the block we are in
        current_block = (math.floor(pos_start / 0.5) * 0.5) + 0.2

        # target is 0.5 away in whatever direction
        target_block = current_block + 0.5 \
            if moving_in_x or moving_in_y \
            else current_block - 0.5

        distance = math.fabs(target_block - pos_start)

        # start movin'!
        cc_factors = self.course_correct_factors()

        self.set_left_motor_velocity(-velocity * cc_factors[0])
        self.set_right_motor_velocity(-velocity * cc_factors[1])

        while not there:

            if self.override_step_forward():
                print("OVERRIDE")
                break

            position = self.get_position(object_name)  # [x, y, z]
            pos_x = position[0]
            pos_y = position[1]

            if moving_in_x is not None:
                if distance - math.fabs(pos_start - pos_x) < 0.001:
                    there = True
            elif moving_in_y is not None:
                if distance - math.fabs(pos_start - pos_y) < 0.001:
                    there = True

            if not there:
                cc_factors = self.course_correct_factors()

                self.set_left_motor_velocity(-velocity * cc_factors[0])
                self.set_right_motor_velocity(-velocity * cc_factors[1])

        # stopphe!
        self.set_left_motor_velocity(0)
        self.set_right_motor_velocity(0)

    def course_correct_factors(self):
        """
        Function to determine what factors to scale each wheel velocity
        by depending on the distance the robo is from each wall. For
        example: if the robo is < 0.175 away from the right wall, left
        wheel velocity will be factored down.

        :return: float, float -> left and right velocity coefficients.
        """
        left_name = "left_proxie"
        right_name = "right_proxie"

        left_factor = 1
        right_factor = 1

        left_reading = self.get_proxie(left_name)
        right_reading = self.get_proxie(right_name)

        # TODO : Add case for when only one sensor is detecting. This will
        #        make the robot enter and exit intersections straighter.
        if left_reading is not None:
            if 0.2 > left_reading > 0.175:
                right_factor = 0.975
            elif 0.175 >= left_reading > 0.1:
                right_factor = 0.925
            elif left_reading <= 0.1:
                right_factor = 0.85

        # only set left_factor if right_factor hasn't been modified
        if right_reading is not None and right_factor > 0.175:
            if 0.2 > right_reading > 0.175:
                left_factor = 0.975
            elif 0.175 >= right_reading > 0.1:
                left_factor = 0.925
            elif right_reading <= 0.1:
                left_factor = 0.85

        return left_factor, right_factor

    def override_step_forward(self):
        """
        Function that returns True or False depending on whether
        there is a wall < 0.15 in front of the Robo.

        :return: bool -> is there a wall < 0.15 in front?
        """
        proxie_name = "front_proxie"

        distance_reading = self.get_proxie(proxie_name)

        if distance_reading is None:
            return False

        elif distance_reading < 0.25:
            return True

        else:
            return False

    def get_vision(self, object_id):
      reading = sim.simxGetObjectHandle(self.__clientID, object_id, sC.simx_opmode_blocking)

      res, resolution, image = sim.simxGetVisionSensorImage(self.__clientID, reading, 0, sC.simx_opmode_streaming)
      print(type(image))
      # 'b' = int : byte size = 1
      image_byte_array = array.array('b', image)
      # size is 5 by 5 array
      im = Image.frombuffer("RGB", (5, 5), image_byte_array, "raw", "RGB", 0, 1)
      im_list = list(im.getdata())
      print(im_list)
      print("[INFO] vision function executed")

      return True

    # def get_vision(self):
    #     """
    #     takes picture with robot's camera (optical / vision sensor)
    #     NOTE: we are using only the top half of the sensor! (this method will crop it)
    #     returns:
    #     - image
    #         [y, x, colors] : numpy array
    #             note: colors=[r,g,b]
    #     """
    #     # print(sim.simxGetVisionSensorImage(self.__clientID,self.visionSensor,0,sC.simx_opmode_blocking)[1:3])
    #     # print(len(sim.simxGetVisionSensorImage(self.__clientID,self.visionSensor,0,sC.simx_opmode_blocking)[1:3]))
    #     [width, height], data = sim.simxGetVisionSensorImage(self.__clientID, self.visionSensor, 0,
    #                                                          sC.simx_opmode_blocking)[1:3]
    #
    #     #   Conversion
    #     # image = np.ndarray((height, width, 3), np.uint8)
    #     # for d in range(len(data) // 2):
    #     for h in range(height):
    #         for w in range(width):
    #             for c in range(3):  # color
    #                 image[h, w, c] = data[c + (width - w) * (h)]  # this doesnt work yet!
    #                 print(h, w, c, '\t', c + (width - w) + width * (h))
    #
    #         # data[d] = 100
    #         # if data[d] > 35: data[d] -= 36
    #     print(type[data[0]])
    #     print([width, height], data)
    #     print()
    #     print(image)
    #     # plt.axis("off")
    #     # plt.imshow(image)
    #     # plt.show()
    #
    #     return 0

    def get_orientation(self, object_name):
        """
        Get the orientation of the specified object.

        :param object_name: str -> the object to get.
        :return: float -> the euler angle g representing the object's orientation.
        """
        if object_name not in self.__scene_objects:
            return None

        handle = self.__scene_objects[object_name]

        return sim.simxGetObjectOrientation(self.__clientID, handle,
                                            -1, sC.simx_opmode_blocking)[1]

    def get_position(self, object_name):
        """
        Get the position of the specified object.

        :param object_name: str -> the object to get.
        :return: list() -> absolute [x, y, z] coordinates of the object.
        """
        if object_name not in self.__scene_objects:
            return None

        handle = self.__scene_objects[object_name]

        return sim.simxGetObjectPosition(self.__clientID, handle,
                                         -1, sC.simx_opmode_blocking)[1]

    def get_proxie(self, proxie_name):
        """
        Get the distance reading for the specified proxie name.

        :param proxie_name: str -> the proxie name.
        :return: float/None -> the distance reading/the proxie is not detecting.
        """

        if proxie_name not in self.__proxies:
            return None

        handle = self.__proxies[proxie_name]

        #  TODO : FAKE NEWS.
        """
        A list that contains:
        item1 (bool): Whether the function was successfully called on the server side
        item2 (number): detection state (0 or 1)
        item3 (number): The distance to the detected point
        item4 (list): The detected point relative to the sensor frame
        item5 (number): The detected object handle
        item6 (list): The normal vector of the detected surface, relative to the sensor frame
        """
        reading = sim.simxReadProximitySensor(self.__clientID, handle,
                                              sC.simx_opmode_blocking)

        print("{} - {}".format(proxie_name, reading[2][2]))
        if reading[1]:
            return reading[2][2]
        else:
            # return None if the sensor is not detecting
            return None

    def set_left_motor_velocity(self, velocity):
        """
        Set the left motor to the specified velocity.

        :param velocity: float -> velocity to set the motor to.
        :return: None
        """
        handle = self.__scene_objects["left_motor"]
        sim.simxSetJointTargetVelocity(self.__clientID, handle,
                                       velocity, sC.simx_opmode_oneshot)

    def set_right_motor_velocity(self, velocity):
        """
        Set the right motor to the specified velocity.

        :param velocity: float -> velocity to set the motor to.
        :return: None
        """
        handle = self.__scene_objects["right_motor"]
        sim.simxSetJointTargetVelocity(self.__clientID, handle,
                                       velocity, sC.simx_opmode_oneshot)

    def raise_arm_left_step(self, step):
        """
        Sets the target position of the left joint to its current position plus the step.

        :param step: int -> distance to move from current position.
        :return: None
        """
        handle = self.__scene_objects["arm_joint_left"]
        step_rad = -step * math.pi / 180

        current_position = self.__get_joint_pos(handle)[1]

        sim.simxSetJointTargetPosition(self.__clientID, handle,
                                       current_position + step_rad, sC.simx_opmode_oneshot)

    def raise_arm_right_step(self, step):
        """
        Sets the target position of the right joint to its current position plus the step.

        :param step: int -> distance to move from current position.
        :return: None
        """
        handle = self.__scene_objects["arm_joint_right"]
        step_rad = -step * math.pi / 180

        current_position = self.__get_joint_pos(handle)[1]

        sim.simxSetJointTargetPosition(self.__clientID, handle,
                                       current_position + step_rad, sC.simx_opmode_oneshot)

    def lower_arm_left_step(self, step):
        """
        Sets the target position of the left joint to its current position minus the step.

        :param step: int -> distance to move from current position.
        :return: None
        """
        handle = self.__scene_objects["arm_joint_left"]
        step_rad = -step * math.pi / 180

        current_position = self.__get_joint_pos(handle)[1]

        sim.simxSetJointTargetPosition(self.__clientID, handle,
                                       current_position - step_rad, sC.simx_opmode_oneshot)

    def lower_arm_right_step(self, step):
        """
        Sets the target position of the right joint to its current position minus the step.

        :param step: int -> distance to move from current position.
        :return: None
        """
        handle = self.__scene_objects["arm_joint_right"]
        step_rad = -step * math.pi / 180

        current_position = self.__get_joint_pos(handle)[1]

        sim.simxSetJointTargetPosition(self.__clientID, handle,
                                       current_position - step_rad, sC.simx_opmode_oneshot)

    def set_arm_right_pos(self, position):
        """
        Sets the target position of the right joint to position.

        :param position: int -> position to move arm to.
        :return: None
        """
        handle = self.__scene_objects["arm_joint_right"]

        sim.simxSetJointTargetPosition(self.__clientID, handle,
                                       position, sC.simx_opmode_oneshot)

    def set_arm_left_pos(self, position):
        """
        Sets the target position of the left joint to position.

        :param position: int -> position to move arm to.
        :return: None
        """
        handle = self.__scene_objects["arm_joint_left"]

        sim.simxSetJointTargetPosition(self.__clientID, handle,
                                       position, sC.simx_opmode_oneshot)

    def generate_maze_in_coppelia(self, maze):
        """
        Calls a child script in CoppeliaSim. This child script places cuboids to
        create the specified maze.

        :param maze: Maze -> object representing the maze to create.
        :return: None
        """
        _ = sim.simxCallScriptFunction(self.__clientID,
                                       "",
                                       sC.sim_scripttype_childscript,
                                       "render@ResizableFloor_5_25",
                                       [maze.length, maze.width],
                                       [maze.length/2, maze.width/2],
                                       maze.reduce(),
                                       bytearray(),
                                       sC.simx_opmode_blocking)

    def close_sim_connection(self):
        """
        Function to close the sim connection.

        :return: None
        """
        # send some data to CoppeliaSim in a non-blocking fashion:
        sim.simxAddStatusbarMessage(self.__clientID, "PEACE OUT CoppeliaSim", sC.simx_opmode_oneshot)

        # make sure that the last command sent out had time to arrive
        sim.simxGetPingTime(self.__clientID)
        # close the connection to CoppeliaSim
        sim.simxFinish(self.__clientID)

    def send_msg(self, msg):
        """
        Print a message in the CoppeliaSim window.

        :param msg: str -> the message to print.
        :return: None
        """
        sim.simxAddStatusbarMessage(self.__clientID, msg, sC.simx_opmode_oneshot)

    def __get_joint_pos(self, handle):
        """
        Get the position of the specified joint.

        :param handle: int -> the handle of the wanted joint.
        :return: float -> the position of the joint in rad.
        """
        return sim.simxGetJointPosition(self.__clientID, handle,
                                        sC.simx_opmode_blocking)

    def __get_scene_objects_dict(self):
        """
        Gets scene objects and maps the handle to the name.

        :return: dict -> object names mapped to integer handles.
        """
        objects_dict = {
            "arm_joint_left": sim.simxGetObjectHandle(self.__clientID, "arm_joint_left",
                                                      sC.simx_opmode_blocking)[1],
            "arm_joint_right": sim.simxGetObjectHandle(self.__clientID, "arm_joint_right",
                                                       sC.simx_opmode_blocking)[1],
            "left_motor": sim.simxGetObjectHandle(self.__clientID, "left_motor",
                                                  sC.simx_opmode_blocking)[1],
            "right_motor": sim.simxGetObjectHandle(self.__clientID, "right_motor",
                                                   sC.simx_opmode_blocking)[1],
            "body": sim.simxGetObjectHandle(self.__clientID, "body",
                                            sC.simx_opmode_blocking)[1]
        }

        proxies_dict = {
            "left_proxie": sim.simxGetObjectHandle(self.__clientID, "left_proxie",
                                                   sC.simx_opmode_blocking)[1],
            "front_proxie": sim.simxGetObjectHandle(self.__clientID, "front_proxie",
                                                    sC.simx_opmode_blocking)[1],
            "right_proxie": sim.simxGetObjectHandle(self.__clientID, "right_proxie",
                                                    sC.simx_opmode_blocking)[1]
        }
        return objects_dict, proxies_dict
