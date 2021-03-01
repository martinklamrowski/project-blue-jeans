import utils.constants as consts


class MapNode(object):

    # def __new__(cls, *args, **kwargs):
    #     print("MapNode created.")
    #     return super(MapNode, cls).__new__(cls)

    def __init__(self, n: bool, e: bool, s: bool, w: bool, j: int, i: int):
        """
        Makes a new MapNode.

        :param n: bool -> wall to the north or not
        :param e: bool -> wall to the east or not
        :param s: bool -> wall to the south or not
        :param w: bool -> wall to the west or not
        :param i: bool -> x position in map
        :param j: bool -> y position in map
        """

        self.walls = {
            consts.NORTH: n,
            consts.EAST: e,
            consts.SOUTH: s,
            consts.WEST: w
        }
        self.j = j
        self.i = i
