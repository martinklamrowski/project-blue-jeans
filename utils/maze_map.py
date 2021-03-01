"""
Will hold information about the maze for the PyRobo.
"""
import time

from .map_node import MapNode
import utils.constants as consts


class MazeMap(object):
    def __init__(self, width: int, height: int, is_empty: bool):
        """
        Makes a map to model a maze (2D grid of nodes).

        :param width: int -> of the map
        :param height: int -> of the map
        :param is_empty: bool -> initialize empty map if True; creates random new map if False
        """

        print("I'm the map, I'm the map.")

        # 2D array is fine for our purposes, only becomes a problem if we have ginormous mazes
        self.__map = [[MapNode(False, False, False, False, j, i) for j in range(height)] for i in range(width)]
        self.width = width
        self.height = height
        if not is_empty:
            self.__generate_fake_map()

    def __str__(self):
        """
        To string utility; prints a rough textual representation of le maze.

        :return: str -> a rough representation of the maze.
        """

        top_row = "   "
        for i in range(self.width):
            top_row += "{} ".format(i)
        maze_rows = [top_row, "  " + "+-" * self.width + "+"]
        for j in range(self.height):
            if j == 0:
                maze_row = ["0  "]
            else:
                maze_row = ["{} |".format(j)]

            for i in range(self.width):
                if self.__map[j][i].walls[consts.EAST]:
                    maze_row.append(" |")
                else:
                    maze_row.append("  ")
            maze_rows.append("".join(maze_row))
            maze_row = ["  +"]
            for i in range(self.width):
                if self.__map[j][i].walls[consts.SOUTH]:
                    maze_row.append("-+")
                else:
                    maze_row.append(" +")
            maze_rows.append("".join(maze_row))
        return "\n".join(maze_rows)

    def set_map_node(self, map_node):
        self.__map[map_node.j][map_node.i] = map_node

    def get_map_node_at_pos(self, j, i):
        return self.__map[j][i]

    def current_shortest_path(self):
        """
        TODO: This should probably be in Robo
        :return: int
        """
        return -1

    def __generate_fake_map(self):
        """
        Doesn't generate for now; reads from predefined file.

        :return: 2D Array<MapNode> -> the generated map
        """

        # 10x10 for now
        with open("misc/maze_rep.txt", "r") as maze_file:
            for j, line in enumerate(maze_file.readlines()):
                for i, node in enumerate(line.split(" ")):
                    self.__map[j][i] = MapNode("n" in node, "e" in node, "s" in node, "w" in node, j, i)
                    # print(self.__str__())
                    # time.sleep(1)



