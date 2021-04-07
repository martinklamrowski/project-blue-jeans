import math
import numpy as np
import sklearn.cluster as sk_cluster

import sim_lib.sim as sim
import sim_lib.simConst as sC

import utils.vec as vec
import utils.constants as consts


class Boundary(object):
    """Class to handle direct communication with CoppeliaSim using the RemoteAPI.
      - Contains some low-level getters and setters for CoppeliaSim scene object attributes.
      - Also contains some higher-level functions to perform an entire routine, like step the Robo forward one block.
    """

    def __init__(self, port):
        """Initialize self.

        :param port: The port value set in CoppeliaSim.
        :type port: int
        """

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
        self.__scene_objects, self.__sensors = self.__get_scene_objects_dict()

    def snap_to_angular_point(self, velocity, angular_point):
        """Function that handles all of the details relating to snapping the Robo's front to the specified angular
        point.

        :param velocity: Velocity to turn at; will actually turn at twice this value.
        :type velocity: float
        :param angular_point: Angular point, probably as defined in constants.py.
        :type angular_point: float
        :return: None
        """
        object_name = "body"
        oriented = False

        # don't turn if we're already snapped to the angular point!
        starting_orientation = vec.euler_g_to_rad(self.get_orientation(object_name)[2])
        if math.fabs(vec.euler_g_to_rad(starting_orientation) - angular_point) < 0.05:
            return

        # start turning
        if vec.get_closer_rotation_direction(starting_orientation, angular_point):
            self.set_left_motor_velocity(velocity)
            self.set_right_motor_velocity(-velocity)
        else:
            self.set_left_motor_velocity(-velocity)
            self.set_right_motor_velocity(velocity)

        while not oriented:
            euler_angles = self.get_orientation(object_name)
            g = euler_angles[2]

            if math.fabs(vec.euler_g_to_rad(g) - angular_point) < 0.05:
                oriented = True

        # stoohpe
        self.set_left_motor_velocity(0)
        self.set_right_motor_velocity(0)

    def step_forward(self, velocity, angular_point):
        """Handles all of the details for moving forward a specified number of steps (blocks).

        :param velocity: Velocity to move forward at.
        :type velocity: float
        :param angular_point: The heading.
        :type angular_point: float
        :return: None
        """
        object_name = "body"

        # need to know how the coordinates will be updating; True for increase
        moving_in_x = None
        moving_in_y = None
        there = False

        if angular_point == 0 or angular_point == math.pi:
            moving_in_y = True if angular_point == 0 else False

        elif angular_point == 3 * math.pi / 2 or angular_point == math.pi / 2:
            moving_in_x = True if angular_point == 3 * math.pi / 2 else False

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
        """Function to determine what factors to scale each wheel velocity by depending on the distance the Robo is from
        each wall. For example: if the robo is < 0.175 away from the right wall, left wheel velocity will be factored
        down.

        :return: Left and right velocity coefficients.
        :rtype: tuple(float, float)
        """
        left_name = "left_proxie"
        right_name = "right_proxie"

        left_factor = 1
        right_factor = 1

        left_reading = self.get_proxie(left_name)
        right_reading = self.get_proxie(right_name)

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
        Function that returns True or False depending on whether there is a wall < 0.15 in front of the Robo.

        :return: Is there a wall < 0.15 in front?
        :rtype: bool
        """
        proxie_name = "front_proxie"

        distance_reading = self.get_proxie(proxie_name)

        if distance_reading is None:
            return False

        elif distance_reading < 0.25:
            return True
        else:
            return False

    def get_vision(self):
        """Function that returns True or False depending on the color it sees.

        :return: True if color matches (blue jeans), else return False.
        :rtype: bool
        """
        # get the handle of the vision sensor
        handle = self.__sensors["ortho_sensor"]

        # get image from vision sensor
        err, res, image = sim.simxGetVisionSensorImage(self.__clientID, handle, 0, sC.simx_opmode_blocking)

        # resize
        image_vector = np.array(image, dtype=np.uint8)
        image_vector.resize([res[0] * res[1], 3])

        percentage_blue = self.__get_blue_prominence(image_vector)
        print(percentage_blue)
        if percentage_blue > 0:
            print("FOUND YA")
            return True

        return False

    def get_orientation(self, object_name):
        """Get the orientation of the specified object.

        :param object_name: Name of the object to get.
        :type object_name: str
        :return: The euler angles [a, b, g]? representing the object's orientation.
        :rtype: list(float)
        """
        if object_name not in self.__scene_objects:
            return None

        handle = self.__scene_objects[object_name]

        return sim.simxGetObjectOrientation(self.__clientID, handle,
                                            -1, sC.simx_opmode_blocking)[1]

    def get_position(self, object_name):
        """Get the position of the specified object.

        :param object_name: Name of the object to get.
        :type object_name: str
        :return: Absolute [x, y, z] coordinates of the object.
        :rtype: list(float)
        """
        if object_name not in self.__scene_objects:
            return None

        handle = self.__scene_objects[object_name]

        return sim.simxGetObjectPosition(self.__clientID, handle,
                                         -1, sC.simx_opmode_blocking)[1]

    def get_proxie(self, proxie_name):
        """Get the distance reading for the specified proxie name.

        :param proxie_name: The proxie name.
        :type proxie_name: str
        :return: The distance reading/the proxie is not detecting. None if not detecting.
        :rtype: float/None
        """

        if proxie_name not in self.__sensors:
            return None

        handle = self.__sensors[proxie_name]

        """
        A list that contains (NOTE: THE COPPELIA DOCS LIE!):
        item1 (bool): Whether the function was successfully called on the server side
        item2 (number): detection state (0 or 1)
        item3 (number): The distance to the detected point --X--> list()
        item4 (list): The detected point relative to the sensor frame
        item5 (number): The detected object handle
        item6 (list): The normal vector of the detected surface, relative to the sensor frame
        """
        reading = sim.simxReadProximitySensor(self.__clientID, handle,
                                              sC.simx_opmode_blocking)

        if reading[1]:
            return reading[2][2]
        else:
            # return None if the sensor is not detecting
            return None

    def set_left_motor_velocity(self, velocity):
        """Set the left motor to the specified velocity.

        :param velocity: Velocity to set the motor to.
        :type velocity: float
        :return: None
        """
        handle = self.__scene_objects["left_motor"]
        sim.simxSetJointTargetVelocity(self.__clientID, handle,
                                       velocity, sC.simx_opmode_oneshot)

    def set_right_motor_velocity(self, velocity):
        """Set the right motor to the specified velocity.

        :param velocity: Velocity to set the motor to.
        :type velocity: float
        :return: None
        """
        handle = self.__scene_objects["right_motor"]
        sim.simxSetJointTargetVelocity(self.__clientID, handle,
                                       velocity, sC.simx_opmode_oneshot)

    def raise_arm_left_step(self, step):
        """Sets the target position of the left joint to its current position plus the step.

        :param step: Distance to move from current position.
        :type step: int
        :return: None
        """
        handle = self.__scene_objects["arm_joint_left"]
        step_rad = -step * math.pi / 180

        current_position = self.__get_joint_pos(handle)

        sim.simxSetJointTargetPosition(self.__clientID, handle,
                                       current_position + step_rad, sC.simx_opmode_oneshot)

    def raise_arm_right_step(self, step):
        """Sets the target position of the right joint to its current position plus the step.

        :param step: Distance to move from current position.
        :type step: int
        :return: None
        """
        handle = self.__scene_objects["arm_joint_right"]
        step_rad = -step * math.pi / 180

        current_position = self.__get_joint_pos(handle)

        sim.simxSetJointTargetPosition(self.__clientID, handle,
                                       current_position + step_rad, sC.simx_opmode_oneshot)

    def lower_arm_left_step(self, step):
        """Sets the target position of the left joint to its current position minus the step.

        :param step: Distance to move from current position.
        :type step: int
        :return: None
        """
        handle = self.__scene_objects["arm_joint_left"]
        step_rad = -step * math.pi / 180

        current_position = self.__get_joint_pos(handle)

        sim.simxSetJointTargetPosition(self.__clientID, handle,
                                       current_position - step_rad, sC.simx_opmode_oneshot)

    def lower_arm_right_step(self, step):
        """Sets the target position of the right joint to its current position minus the step.

        :param step: Distance to move from current position.
        :type step: int
        :return: None
        """
        handle = self.__scene_objects["arm_joint_right"]
        step_rad = -step * math.pi / 180

        current_position = self.__get_joint_pos(handle)

        sim.simxSetJointTargetPosition(self.__clientID, handle,
                                       current_position - step_rad, sC.simx_opmode_oneshot)

    def set_arm_right_pos(self, position):
        """Sets the target position of the right joint to position.

        :param position: Position to move arm to.
        :type position: int
        :return: None
        """
        handle = self.__scene_objects["arm_joint_right"]

        sim.simxSetJointTargetPosition(self.__clientID, handle,
                                       position, sC.simx_opmode_oneshot)

    def set_arm_left_pos(self, position):
        """Sets the target position of the left joint to position.

        :param position: Position to move arm to.
        :type position: int
        :return: None
        """
        handle = self.__scene_objects["arm_joint_left"]

        sim.simxSetJointTargetPosition(self.__clientID, handle,
                                       position, sC.simx_opmode_oneshot)

    def generate_maze_in_coppelia(self, maze):
        """Calls a child script in CoppeliaSim. This child script places cuboids to create the specified maze.

        :param maze: Maze object representing the maze to create.
        :type maze: class: utils.maze.Maze
        :return: None
        """
        _ = sim.simxCallScriptFunction(self.__clientID,
                                       "",
                                       sC.sim_scripttype_childscript,
                                       "render@ResizableFloor_5_25",
                                       [maze.length, maze.width],
                                       [maze.length / 2, maze.width / 2],
                                       maze.reduce(),
                                       bytearray(),
                                       sC.simx_opmode_blocking)

    def close_sim_connection(self):
        """Function to close the sim connection.

        :return: None
        """
        # send some data to CoppeliaSim in a non-blocking fashion:
        sim.simxAddStatusbarMessage(self.__clientID, "PEACE OUT CoppeliaSim", sC.simx_opmode_oneshot)

        # make sure that the last command sent out had time to arrive
        sim.simxGetPingTime(self.__clientID)
        # close the connection to CoppeliaSim
        sim.simxFinish(self.__clientID)

    def send_msg(self, msg):
        """Print a message in the CoppeliaSim window.

        :param msg: The message to print.
        :type msg: str
        :return: None
        """
        sim.simxAddStatusbarMessage(self.__clientID, msg, sC.simx_opmode_oneshot)

    def __get_blue_prominence(self, image_vector):

        estimator = sk_cluster.KMeans(n_clusters=2).fit(image_vector)
        cluster = estimator.labels_
        labels = estimator.cluster_centers_

        # get frequency of each label
        hist, _ = np.histogram(cluster, bins=np.arange(0, len(np.unique(cluster)) + 1))
        hist = hist.astype("float")
        hist /= hist.sum()

        percentage_blue = 0
        print(estimator.cluster_centers_)
        for p, label in zip(hist, labels):
            print((p, label.astype("uint8").tolist()))

            if vec.is_similar_colour(label.astype("uint8").tolist(), consts.JEANS_COLOUR):
                percentage_blue = p

        return percentage_blue

    def __get_joint_pos(self, handle):
        """Get the position of the specified joint.

        :param handle: The handle of the wanted joint.
        :type handle: int
        :return: The position of the joint in rad.
        :rtype: float
        """
        return sim.simxGetJointPosition(self.__clientID, handle,
                                        sC.simx_opmode_blocking)[1]

    def __get_scene_objects_dict(self):
        """Gets scene objects and maps the handle to the name.

        :return: The objects and proxies dictionaries respectively with object names mapped to integer handles.
        :rtype: tuple( dict(str : int), dict(str : int) )
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

        sensors_dict = {
            "left_proxie": sim.simxGetObjectHandle(self.__clientID, "left_proxie",
                                                   sC.simx_opmode_blocking)[1],
            "front_proxie": sim.simxGetObjectHandle(self.__clientID, "front_proxie",
                                                    sC.simx_opmode_blocking)[1],
            "right_proxie": sim.simxGetObjectHandle(self.__clientID, "right_proxie",
                                                    sC.simx_opmode_blocking)[1],
            "ortho_sensor": sim.simxGetObjectHandle(self.__clientID, "ortho",
                                                    sC.simx_opmode_blocking)[1]
        }
        return objects_dict, sensors_dict
