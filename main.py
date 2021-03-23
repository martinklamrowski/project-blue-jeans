import argparse
import time
import keyboard

from utils.boundary import Boundary
from utils.maze_map import MazeMap
from pyrobo.robo import Robo
import utils.constants as consts

ap = argparse.ArgumentParser()

ap.add_argument("-p", "--port", required=True, help="Add a port man.")
args = vars(ap.parse_args())


def main():
    # this is a test; the robo doesn't know about this maze, it will have a different one internally
    fake_maze = MazeMap(10, 10, False)
    print(fake_maze)

    # TODO: I think the Boundary instance should eventually be moved to Robo, could also
    #       just pass b to Robo() - that's what i'll do for now.
    b = Boundary(int(args["port"]))

    # b.send_msg("heyo!")

    # print(b.get_proxy())

    # TODO : Change Boundary-Robo relationship to composition, not aggregation.
    robo = Robo(boundary=b, testing=False)
    # robo.print_map()

    # main routine
    while True:
        if keyboard.is_pressed("u"):
            robo.raise_arm_step(consts.LEFT_ARM)
        elif keyboard.is_pressed("j"):
            robo.lower_arm_step(consts.LEFT_ARM)
        if keyboard.is_pressed("i"):
            robo.raise_arm_step(consts.RIGHT_ARM)
        elif keyboard.is_pressed("k"):
            robo.lower_arm_step(consts.RIGHT_ARM)

        if keyboard.is_pressed("w"):
            if keyboard.is_pressed("a"):
                robo.accelerate_forward(turn="left")
            elif keyboard.is_pressed("d"):
                robo.accelerate_forward(turn="right")
            else:
                robo.accelerate_forward(turn=None)
        elif keyboard.is_pressed("s"):
            if keyboard.is_pressed("a"):
                robo.accelerate_backward(turn="left")
            elif keyboard.is_pressed("d"):
                robo.accelerate_backward(turn="right")
            else:
                robo.accelerate_backward(turn=None)
        else:
            robo.decelerate()


    #     robo.move_to_next(fake_maze)
    #     robo.print_map()
    #     print("Robo's Current Position: {}j, {}i".format(robo.pos_j, robo.pos_i))
    #     time.sleep(1)


if __name__ == "__main__":
    main()
