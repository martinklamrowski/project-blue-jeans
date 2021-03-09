import sim_lib.sim as sim
import sim_lib.simConst as sC
import math
import numpy as np


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
            if dists[n] < 0.05: dists[n] = None

        return dists

    """
    takes picture with robot's camera (optical / vision sensor)
    returns:
    - image
        [y, x, colors] : numpy array
            note: colors=[r,g,b]
    """
    def get_vision(self):

        [width,height], data = sim.simxGetVisionSensorImage(self.__clientID,self.visionSensor,0,sC.simx_opmode_blocking)[1:2]

        return 0

    def set_left_motor(self):
        print(self.motors)

    def set_right_motor(self):
        print(self.motors)


if __name__ == "__main__":
    b = Boundary(8008)
    b.send_msg("heyo!")
    b.get_vision()
