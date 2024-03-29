import argparse

from pyrobo.boundary import Boundary
from pyrobo.navigation import Navigation
from pyrobo.robo import Robo
from utils.maze import Maze

ap = argparse.ArgumentParser(description="Go Robo go!")
ap.add_argument("-p", "--port", required=True, help="Add a port man.")
ap.add_argument("-d", "--dim", required=True, type=int, nargs=2,
                help="Give me the dimensions man <-d L W>.")
args = vars(ap.parse_args())


def main():
    """
    Simulation setup. Robot is created along with the necessary composing objects.

    Possible usages:
     - Enable manual to start with manual remote control.
     - Disable lines 30-33 for no maze generation.
     - Disable line 32 to generate an empty maze.
    """
    b = Boundary(int(args["port"]))
    n = Navigation(h=int(args["dim"][0]), w=int(args["dim"][1]))

    robo = Robo(boundary=b, nav=n, manual=False)

    # generate maze
    maze = Maze(length=int(args["dim"][0]), width=int(args["dim"][1]), opening="west")
    maze.generate_maze()
    maze.generate_object()
    b.generate_maze_in_coppelia(maze=maze)

    # go robo go
    robo.run()


if __name__ == "__main__":
    main()
