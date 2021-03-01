from utils.maze_map import MazeMap
from utils.map_node import MapNode

# from utils.boundary import Boundary


class Robo(object):
    def __init__(self):
        print("Well hello there.")

        self.maze_map = MazeMap(10, 10, True)

    def print_map(self):
        print(self.maze_map)
