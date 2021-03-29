import argparse
import time

from utils.boundary import Boundary
from utils.maze import Maze
from pyrobo.robo import Robo
import utils.constants as consts

ap = argparse.ArgumentParser()

ap.add_argument("-p", "--port", required=True, help="Add a port man.")
args = vars(ap.parse_args())


def main():

    # TODO: I think the Boundary instance should eventually be moved to Robo, could also
    #       just pass b to Robo() - that's what i'll do for now.
    b = Boundary(int(args["port"]))
    robo = Robo(boundary=b, testing=False)

    # generate maze
    maze = Maze(length=18, width=18, opening="west")
    maze.generate_maze()
    maze.generate_object() # TODO : Figure this out.
    b.generate_maze_in_coppelia(maze=maze)

    # go robo go
    robo.run()


if __name__ == "__main__":
    main()
