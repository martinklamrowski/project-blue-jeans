import argparse
import time

from utils.boundary import Boundary
from utils.maze_map import MazeMap
from pyrobo.robo import Robo

ap = argparse.ArgumentParser()

ap.add_argument("-p", "--port", required=True, help="Add a port man.")
args = vars(ap.parse_args())


def main():
    # this is a test; the robo doesn't know about this maze, it will have a different one internally
    fake_maze = MazeMap(10, 10, False)
    print(fake_maze)

    # TODO: I think the Boundary instance should eventually be moved to Robo, could also
    #       just pass b to Robo() - that's what i'll do for now.
    # b = Boundary(int(args["port"]))

    # b.send_msg("heyo!")

    # print(b.get_proxy())

    robo = Robo(boundary=None, testing=True)
    robo.print_map()

    # main routine
    while True:
        robo.move_to_next(fake_maze)
        robo.print_map()
        print("Robo's Current Position: {}j, {}i".format(robo.pos_j, robo.pos_i))
        time.sleep(1)


    # b.print(get_vision())
    #b.close_sim_connection()

    #
    #     time.sleep(2)
    #
    #     # # retrieve data in a blocking fashion (i.e. a service call):
    #     # # res, objs = sim.simxGetObjects(clientID, sim.sim_handle_all, sim.simx_opmode_blocking)
    #     #
    #     # # there are no vision sensors i.e. this will fail
    #     # vision_sensors = (sim.simxGetObjectHandle(clientID, "Vision_sensor1",
    #     #                                           sim.simx_opmode_blocking)[1],
    #     #                   sim.simxGetObjectHandle(clientID, "Vision_sensor0",
    #     #                                           sim.simx_opmode_blocking)[1],
    #     #                   sim.simxGetObjectHandle(clientID, "Vision_sensor",
    #     #                                           sim.simx_opmode_blocking)[1])
    #     #
    #     # left_joint = sim.simxGetObjectHandle(clientID, "Revolute_joint0",
    #     #                                      sim.simx_opmode_blocking)[1]
    #     #
    #     # right_joint = sim.simxGetObjectHandle(clientID, "Revolute_joint",
    #     #                                       sim.simx_opmode_blocking)[1]
    #     #
    #     # while True:
    #     #     # left reading
    #     #     ret_code0, det_state0, data0 = sim.simxReadVisionSensor(clientID, vision_sensors[0],
    #     #                                                             sim.simx_opmode_blocking)
    #     #
    #     #     # right reading
    #     #     ret_code2, det_state2, data2 = sim.simxReadVisionSensor(clientID, vision_sensors[2],
    #     #                                                             sim.simx_opmode_blocking)
    #     #
    #     #     if ret_code0 != -1 and det_state0 != -1:
    #     #         print(ret_code0, det_state0, data0)  #
    #     #         print("{} {}".format(data0[0][10], data0[0][10] < 0.3))
    #     #
    #     #         sim.simxSetJointTargetVelocity(clientID,
    #     #                                        right_joint,
    #     #                                        0.1 if (data0[0][10] < 0.3) else NOM_LIN_VEL,
    #     #                                        sim.simx_opmode_oneshot)
    #     #
    #     #     if ret_code2 != -1 and det_state2 != -1:
    #     #         print(ret_code2, det_state2, data2)  #
    #     #         print("{} {}".format(data2[0][10], data2[0][10] < 0.3))
    #     #         sim.simxSetJointTargetVelocity(clientID,
    #     #                                        left_joint,
    #     #                                        0.1 if (data2[0][10] < 0.3) else NOM_LIN_VEL,
    #     #                                        sim.simx_opmode_oneshot)
    #
    #     # send some data to CoppeliaSim in a non-blocking fashion:
    #     sim.simxAddStatusbarMessage(clientID, "Hello CoppeliaSim!", sim.simx_opmode_oneshot)
    #
    #     # make sure that the last command sent out had time to arrive
    #     sim.simxGetPingTime(clientID)
    #
    #     # close the connection to CoppeliaSim
    #     sim.simxFinish(clientID)
    # else:
    #     print("Failed connecting to remote API server")


if __name__ == "__main__":
    main()
