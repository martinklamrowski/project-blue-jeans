import sim_lib.sim as sim
import sim_lib.simConst as sC
import math
import numpy as np
import matplotlib.pyplot as plt
import time

import math


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
        self.proxySensors = (sim.simxGetObjectHandle(self.__clientID, "LeftProximitySensor", sC.simx_opmode_blocking)[1],
                             sim.simxGetObjectHandle(self.__clientID, "FrontProximitySensor", sC.simx_opmode_blocking)[1],
                             sim.simxGetObjectHandle(self.__clientID, "RightProximitySensor", sC.simx_opmode_blocking)[1])
        self.motors = (sim.simxGetObjectHandle(self.__clientID, "LeftJoint", sC.simx_opmode_blocking)[1],
                       sim.simxGetObjectHandle(self.__clientID, "RightJoint", sC.simx_opmode_blocking)[1])
        self.visionSensor = sim.simxGetObjectHandle(self.__clientID, "VisionSensor", sC.simx_opmode_blocking)[1]

    """
    ALWAYS RUN LAST
    """
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

    """
    prints string (msg) in coppelia command window
    """
    def send_msg(self, msg):
        sim.simxAddStatusbarMessage(self.__clientID, msg, sC.simx_opmode_oneshot)

    """
    reads poximity sensor data
    return: 
    - lengths to nearest object
        [LeftSensor, FrontSensor, RightSensor] : List
            each cell contains distance to nearest object (in meters)
            OR None value if nothing has been detected
            
    """
    def get_proxys(self):
        dists = [0,0,0]
        dists[0], temp, temp = sim.simxReadProximitySensor(self.__clientID, self.proxySensors[0], sC.simx_opmode_blocking)[2:5]
        dists[1], temp, temp = sim.simxReadProximitySensor(self.__clientID, self.proxySensors[1], sC.simx_opmode_blocking)[2:5]
        dists[2], temp, temp = sim.simxReadProximitySensor(self.__clientID, self.proxySensors[2], sC.simx_opmode_blocking)[2:5]
        del temp
        for n in range(len(dists)):
            dists[n] = math.sqrt((dists[n][0]) ** 2 + (dists[n][1]) ** 2 + (dists[n][2]) ** 2)
            # if dists[n]<2.5 and dists[n]>=1.5:  dists[n] = 2
            if dists[n]<1.5 and dists[n]>=0.35:  dists[n] = 1
            elif dists[n]<0.35 and dists[n]>=0.05: dists[n] = 0
            # elif dists[n] < 0.05: dists[n] = None
            else: dists[n] = None
        return dists

    """
    takes picture with robot's camera (optical / vision sensor)     
    NOTE: we are using only the top half of the sensor! (this method will crop it)
    returns:
    - image
        [y, x, colors] : numpy array
            note: colors=[r,g,b]
    """
    def get_vision(self):
        # print(sim.simxGetVisionSensorImage(self.__clientID,self.visionSensor,0,sC.simx_opmode_blocking)[1:3])
        # print(len(sim.simxGetVisionSensorImage(self.__clientID,self.visionSensor,0,sC.simx_opmode_blocking)[1:3]))
        [width,height], data = sim.simxGetVisionSensorImage(self.__clientID,self.visionSensor,0,sC.simx_opmode_blocking)[1:3]

        #   Conversion
        image = np.ndarray((height,width,3), np.uint8)
        # for d in range(len(data) // 2):
        for h in range(height):
            for w in range(width):
                for c in range(3): # color
                    image[h,w,c] = data[c + (width-w) * (h)]   # this doesnt work yet!
                    print(h,w,c, '\t', c + (width-w) + width * (h))

            # data[d] = 100
            # if data[d] > 35: data[d] -= 36
        print(type[data[0]])
        print([width,height], data)
        print()
        print(image)
        plt.axis("off")
        plt.imshow(image)
        plt.show()

        return 0

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

        current_position = self.__get_joint_pos(handle)

        sim.simxSetJointTargetPosition(self.__clientID, handle,
                                       current_position[1] + step_rad, sC.simx_opmode_oneshot)

    def raise_arm_right_step(self, step):
        """
        Sets the target position of the right joint to its current position plus the step.
        :param step: int -> Distance to move from current position.
        :return: None
        """
        handle = self.__scene_objects["arm_joint_right"]
        step_rad = -step * math.pi / 180

        current_position = self.__get_joint_pos(handle)

        sim.simxSetJointTargetPosition(self.__clientID, handle,
                                       current_position[1] + step_rad, sC.simx_opmode_oneshot)

    def lower_arm_left_step(self, step):
        """
        Sets the target position of the left joint to its current position minus the step.
        :param step: int -> Distance to move from current position.
        :return: None
        """
        handle = self.__scene_objects["arm_joint_left"]
        step_rad = -step * math.pi / 180

        current_position = self.__get_joint_pos(handle)

        sim.simxSetJointTargetPosition(self.__clientID, handle,
                                       current_position[1] - step_rad, sC.simx_opmode_oneshot)

    def lower_arm_right_step(self, step):
        """
        Sets the target position of the right joint to its current position minus the step.
        :param step: int -> Distance to move from current position.
        :return: None
        """
        handle = self.__scene_objects["arm_joint_right"]
        step_rad = -step * math.pi / 180

        current_position = self.__get_joint_pos(handle)

        sim.simxSetJointTargetPosition(self.__clientID, handle,
                                       current_position[1] - step_rad, sC.simx_opmode_oneshot)

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
            "FrontProximitySensor": sim.simxGetObjectHandle(self.__clientID, "FrontProximitySensor",
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
                                                   sC.simx_opmode_blocking)[1]
        }

        return objects_dict

if __name__ == "__main__":
    b = Boundary(8008)
    b.send_msg("heyo!")
    # b.get_vision()
    while True:
        l,c,r = b.get_proxys()
        b.send_msg("left: " + str(l) + "\tcenter:" + str(c) + "\tright:" + str(r))
        if c == 0:
            b.send_msg("STOP!!!!!!!!!!!!!!")
            break

    b.close_sim_connection()
