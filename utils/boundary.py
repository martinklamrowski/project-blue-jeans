import math

import sim_lib.sim as sim
import sim_lib.simConst as sC

import utils.vec as vec # TODO : Move this dependency to robo.py.


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

    # TODO : ?.
    def send_msg(self, msg):
        sim.simxAddStatusbarMessage(self.__clientID, msg, sC.simx_opmode_oneshot)

    # TODO : Fugly.
    def get_proxy(self):
        q = [0, 0]
        q[0], temp, q[1] = sim.simxReadProximitySensor(self.__clientID, self.proxySensors[0],
                                                       sC.simx_opmode_blocking)[2:5]
        return q[1]

    # TODO : Fugly.
    def get_vision(self):
        ret_code, det_state, data = sim.simxReadVisionSensor(self.__clientID, self.visionSensor,
                                                             sC.simx_opmode_blocking)
        # print(type(data[0]))
        # print(data[0])
        # print(len(data[0]))
        return 0

    def turn_right_on_spot(self, vel, angular_point):
        r_handle = self.__scene_objects["right_motor"]
        l_handle = self.__scene_objects["left_motor"]
        b_handle = self.__scene_objects["body"]

        orientated = False
        euler_angles = sim.simxGetObjectOrientation(self.__clientID, b_handle, -1, sC.simx_opmode_blocking)[1]
        prev_g = euler_angles[2]

        starting_rotation = vec.euler_g_to_rad(prev_g)
        rotation = 0

        # start turning
        self.set_left_motor_velocity(vel)
        self.set_right_motor_velocity(-vel)

        while not orientated:
            euler_angles = sim.simxGetObjectOrientation(self.__clientID, b_handle, -1, sC.simx_opmode_blocking)[1]
            g = euler_angles[2]
            dg = g - prev_g
            print(euler_angles)

            if dg >= 0:
                dg = vec.euler_g_to_rad(dg)
            else:
                dg = vec.euler_g_to_rad(dg)
            rotation += dg
            prev_g = g

            """
            absolute rotation angle of the robot ,and mod it with math.pi/2, 
            keep rotating until the difference meets a small value like math.pi/180!
            """
            if math.fabs(rotation) > angular_point - math.fabs(starting_rotation):
                orientated = True

        # stohp
        self.set_left_motor_velocity(0)
        self.set_right_motor_velocity(0)

        # sim.simxSetJointTargetPosition(self.__clientID, r_handle,
        #                                r_current_position + step_rad, sC.simx_opmode_oneshot)
        #
        # sim.simxSetJointTargetPosition(self.__clientID, l_handle,
        #                                l_current_position - step_rad, sC.simx_opmode_oneshot)

    def set_left_motor_velocity(self, vel):
        handle = self.__scene_objects["left_motor"]
        sim.simxSetJointTargetVelocity(self.__clientID, handle, vel, sC.simx_opmode_oneshot)

    def set_right_motor_velocity(self, vel):
        handle = self.__scene_objects["right_motor"]
        sim.simxSetJointTargetVelocity(self.__clientID, handle, vel, sC.simx_opmode_oneshot)

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
        return sim.simxGetJointPosition(self.__clientID, handle, sC.simx_opmode_blocking)

    def __get_scene_objects_dict(self):
        """
        Gets scene objects and maps the handle to the name.
        TODO : Is there a way to make this one call? If there is
               I can't find it.
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
