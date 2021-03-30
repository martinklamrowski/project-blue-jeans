import math
import time

import sim_lib.sim as sim
import sim_lib.simConst as sC

import utils.vec as vec  # TODO : Move this dependency to robo.py.


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
        self.__scene_objects = self.__get_scene_objects_dict()
        self.res, self.objs = sim.simxGetObjects(self.__clientID, sC.sim_handle_all, sC.simx_opmode_blocking)
        self.proxySensors = (
            sim.simxGetObjectHandle(self.__clientID, "LeftProximitySensor", sC.simx_opmode_blocking)[1],
            sim.simxGetObjectHandle(self.__clientID, "FrontProximitySensor", sC.simx_opmode_blocking)[1],
            sim.simxGetObjectHandle(self.__clientID, "RightProximitySensor", sC.simx_opmode_blocking)[1])
        self.motors = (sim.simxGetObjectHandle(self.__clientID, "LeftJoint", sC.simx_opmode_blocking)[1],
                       sim.simxGetObjectHandle(self.__clientID, "RightJoint", sC.simx_opmode_blocking)[1])
        self.visionSensor = sim.simxGetObjectHandle(self.__clientID, "VisionSensor", sC.simx_opmode_blocking)[1]

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
        prints string (msg) in coppelia command window
        """
        sim.simxAddStatusbarMessage(self.__clientID, msg, sC.simx_opmode_oneshot)

    def generate_maze_in_coppelia(self, maze):
        # TODO : UGLY AS FUHHHH.

        emptyBuff = bytearray()
        res, retInts, retFloats, retStrings, retBuffer = sim.simxCallScriptFunction(self.__clientID,
                                                                                    "",
                                                                                    sC.sim_scripttype_childscript,
                                                                                    "render@ResizableFloor_5_25",
                                                                                    [maze.length, maze.width],
                                                                                    [],
                                                                                    maze.reduce(),
                                                                                    emptyBuff,
                                                                                    sC.simx_opmode_blocking)

        # if res == sC.simx_return_ok:
        #     maze.render()
        #     # print ('Dummy handle: ',retInts[0]) # display the reply from CoppeliaSim (in this case, the handle of the created dummy)
        # else:
        #     print("Remote function call failed.")

    def get_proxys(self):
        """
        reads poximity sensor data
        return:
        - lengths to nearest object
            [LeftSensor, FrontSensor, RightSensor] : List
                each cell contains distance to nearest object (in meters)
                OR None value if nothing has been detected

        """
        dists = [0, 0, 0]
        dists[0], temp, temp = sim.simxReadProximitySensor(self.__clientID, self.proxySensors[0],
                                                           sC.simx_opmode_blocking)[2:5]
        dists[1], temp, temp = sim.simxReadProximitySensor(self.__clientID, self.proxySensors[1],
                                                           sC.simx_opmode_blocking)[2:5]
        dists[2], temp, temp = sim.simxReadProximitySensor(self.__clientID, self.proxySensors[2],
                                                           sC.simx_opmode_blocking)[2:5]
        del temp
        for n in range(len(dists)):
            dists[n] = math.sqrt((dists[n][0]) ** 2 + (dists[n][1]) ** 2 + (dists[n][2]) ** 2)  # get range from (y,x,z)
            # if dists[n]<2.5 and dists[n]>=1.5:  dists[n] = 2
            if dists[n] < 1.5 and dists[n] >= 0.35:
                dists[n] = 1  # get how many blocksaway
            elif dists[n] < 0.35 and dists[n] >= 0.05:
                dists[n] = 0
            # elif dists[n] < 0.05: dists[n] = None
            else:
                dists[n] = None
        return dists

    def get_vision(self):
        return False
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
        if object_name not in self.__scene_objects:
            return None

        handle = self.__scene_objects[object_name]

        return sim.simxGetObjectOrientation(self.__clientID, handle,
                                            -1, sC.simx_opmode_blocking)[1]

    def get_position(self, object_name):
        if object_name not in self.__scene_objects:
            return None

        handle = self.__scene_objects[object_name]

        return sim.simxGetObjectPosition(self.__clientID, handle,
                                         -1, sC.simx_opmode_blocking)[1]

    def snap_to_angular_point(self, velocity, angular_point):
        object_name = "body"
        oriented = False

        # don't turn if we're already snapped to the angular point!
        starting_orientation = vec.euler_g_to_rad(self.get_orientation(object_name)[2])
        if math.fabs(vec.euler_g_to_rad(starting_orientation) - angular_point) < 0.01:
            return

        # start turning
        self.set_left_motor_velocity(velocity)
        self.set_right_motor_velocity(-velocity)

        while not oriented:
            euler_angles = self.get_orientation(object_name)
            g = euler_angles[2]

            if math.fabs(vec.euler_g_to_rad(g) - angular_point) < 0.01:
                oriented = True

        # stoohpe
        self.set_left_motor_velocity(0)
        self.set_right_motor_velocity(0)

    def step_forward(self, velocity, distance, angular_point):
        """
        Handles all of the details for moving forward a specified number
        of steps (blocks).
        :param velocity: float -> velocity to move forward at.
        :param distance: float -> distance to move forward.
        :param angular_point: float -> the heading.
        :return: None
        """
        # GPS baby!
        # but this actually isn't that bad; if need be, presumably
        # the 'getPosition' method could be redone by some big brain
        # who has time and money to implement some SpaceX level gyroscopes
        # and stuff to get the orientation and location of the robot -
        # yay for decoupled code!
        object_name = "body"

        # need to know how the coordinates will be updating
        moving_in_x = False
        moving_in_y = False
        there = False

        if angular_point == 0 or angular_point == math.pi:
            # north or south
            moving_in_y = True
            print("north or south")
        elif angular_point == 3 * math.pi/2 or angular_point == math.pi/2:
            # east or west
            moving_in_x = True
            print("east or west")

        pos_start = self.get_position(object_name)
        if moving_in_x:
            pos_start = pos_start[0]  # working with x
        elif moving_in_y:
            pos_start = pos_start[1]  # working with y

        # start movin'!
        self.set_left_motor_velocity(-velocity)
        self.set_right_motor_velocity(-velocity)

        while not there:
            position = self.get_position(object_name)  # [x, y, z]
            pos_x = position[0]
            pos_y = position[1]

            if moving_in_x:
                if distance - math.fabs(pos_start - pos_x) < 0.01:
                    there = True
            elif moving_in_y:
                if distance - math.fabs(pos_start - pos_y) < 0.01:
                    there = True

        # stopphe!
        self.set_left_motor_velocity(0)
        self.set_right_motor_velocity(0)


    def set_left_motor_velocity(self, velocity):
        handle = self.__scene_objects["left_motor"]
        sim.simxSetJointTargetVelocity(self.__clientID, handle,
                                       velocity, sC.simx_opmode_oneshot)

    def set_right_motor_velocity(self, velocity):
        handle = self.__scene_objects["right_motor"]
        sim.simxSetJointTargetVelocity(self.__clientID, handle,
                                       velocity, sC.simx_opmode_oneshot)

    def raise_arm_left_step(self, step):
        """
        Sets the target position of the left joint to its current position plus the step.
        :param step: int -> Distance to move from current position.
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
        :param step: int -> Distance to move from current position.
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
        :param step: int -> Distance to move from current position.
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
        :param step: int -> Distance to move from current position.
        :return: None
        """
        handle = self.__scene_objects["arm_joint_right"]
        step_rad = -step * math.pi / 180

        current_position = self.__get_joint_pos(handle)[1]

        sim.simxSetJointTargetPosition(self.__clientID, handle,
                                       current_position - step_rad, sC.simx_opmode_oneshot)

    def reset_arm_right_pos(self):
        """
        Sets the target position of the left joint to 0.
        :return: None
        """
        handle = self.__scene_objects["arm_joint_right"]

        sim.simxSetJointTargetPosition(self.__clientID, handle,
                                       0, sC.simx_opmode_oneshot)

    def reset_arm_left_pos(self):
        """
        Sets the target position of the left joint to 0.
        :return: None
        """
        handle = self.__scene_objects["arm_joint_left"]

        sim.simxSetJointTargetPosition(self.__clientID, handle,
                                       0, sC.simx_opmode_oneshot)

    def __get_joint_pos(self, handle):
        return sim.simxGetJointPosition(self.__clientID, handle,
                                        sC.simx_opmode_blocking)

    def __get_scene_objects_dict(self):
        """
        Gets scene objects and maps the handle to the name.
        :return: dict -> Object names mapped to integer handles.
        """
        objects_dict = {
            "LeftProximitySensor": sim.simxGetObjectHandle(self.__clientID, "LeftProximitySensor",
                                                           sC.simx_opmode_blocking)[1],
            "distance_proxie": sim.simxGetObjectHandle(self.__clientID, "distance_proxie",
                                                       sC.simx_opmode_blocking)[1],
            "RightProximitySensor": sim.simxGetObjectHandle(self.__clientID, "RightProximitySensor",
                                                            sC.simx_opmode_blocking)[1],
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
        return objects_dict
