import sim

class Boundary:

    clientID, proxySenors, motors, visionSensor = 0,0,0,0

    def initSimConnection(self, port):
    #### global clientID, res, objs, proxySenors, motors, visionSensor

        # connect to CoppeliaSim
        sim.simxFinish(-1)  # just in case, close all opened connections
        self.clientID = sim.simxStart("127.0.0.1", port, True, True, 5000, 0)  # connect to CoppeliaSim
        if self.clientID == -1: return False

        # init robot parts
        res, objs = sim.simxGetObjects(self.clientID, sim.sim_handle_all, sim.simx_opmode_blocking)
        self.proxySensors = (sim.simxGetObjectHandle(self.clientID, "LeftProximitySensor", sim.simx_opmode_blocking)[1],
                      sim.simxGetObjectHandle(self.clientID, "FrontProximitySensor", sim.simx_opmode_blocking)[1],
                      sim.simxGetObjectHandle(self.clientID, "RightProximitySensor", sim.simx_opmode_blocking)[1])
        self.motors = (sim.simxGetObjectHandle(self.clientID, "LeftJoint", sim.simx_opmode_blocking)[1],
                  sim.simxGetObjectHandle(self.clientID, "RightJoint", sim.simx_opmode_blocking)[1])
        self.visionSensor = sim.simxGetObjectHandle(self.clientID, "VisionSensor", sim.simx_opmode_blocking)[1]

        return True

    def closeSimConnection(self):
        # send some data to CoppeliaSim in a non-blocking fashion:
        # sim.simxAddStatusbarMessage(clientID, "PEACE OUT CoppeliaSim", sim.simx_opmode_oneshot)

        # make sure that the last command sent out had time to arrive
        sim.simxGetPingTime(self.clientID)

        # close the connection to CoppeliaSim
        sim.simxFinish(self.clientID)

    def sendMsg(self, msg):
        sim.simxAddStatusbarMessage(self.clientID, msg, sim.simx_opmode_oneshot)

    def getProxy(self):
        q = [0,0]
        q[0], temp, q[1] = sim.simxReadProximitySensor(self.clientID, self.proxySensors[0], sim.simx_opmode_blocking)[2:5]
        return (q[1])

    def getVision(self):
        ret_code, det_state, data = sim.simxReadVisionSensor(self.clientID, self.visionSensor, sim.simx_opmode_blocking)
        # print(type(data[0]))
        # print(data[0])
        # print(len(data[0]))
        return 0

    def setMotorL(self):
        print(self.motors)

    def setMotorR(self):
        print(self.motors)