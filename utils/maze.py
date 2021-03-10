from random import seed, randint
from colorama import init, Fore

class Maze():
    def __init__(self, length, width, opening):
        """Constructor

        Keyword arguments:
        length -- Length of maze
        width -- Width of maze
        opening -- The wall for which an opening should be made {east, west, north, south}
        """
        # Maze boundaries
        self.length = length
        self.width = width
        self.maze = [] # Maze map
        self.CELL = 'c'
        self.WALL = 'w'
        self.UNVISITED_CELL = 'u'
        self.OBJECT = 'O'
        self.opening = opening # Opening in the maze

    def render(self):
        """Pretty prints maze."""
        init()
        for row in range(self.length):
            for col in range(self.width):
                if (self.maze[col][row] == self.CELL):
                    print(Fore.WHITE, f'{self.maze[col][row]}' + ' ', end='')
                elif (self.maze[col][row] == self.WALL):
                    print(Fore.RED, f'{self.maze[col][row]}' + ' ', end='')
                elif (self.maze[col][row] == self.OBJECT):
                    print(Fore.YELLOW, f'{self.maze[col][row]}' + ' ', end='')    
            print()

    def initialize(self):
        """Initializes maze"""
        for _ in range(self.width):
            row = []
            for _ in range(self.length):
                row.append(self.UNVISITED_CELL)
            self.maze.append(row)

    def get_cartesian_surroundings(self, x, y):
        """Returns the coordinates of the neighbouring blocks (by edges, not vertices).

        Keyword arguments:
        x -- x-coordinate of block
        y -- y-coordinate of block
        """
        surroundings = {}

        surroundings["west"] = [x-1, y] if x > 0 else []
        surroundings["east"] = [x+1, y] if x < self.width-1 else []
        surroundings["north"] = [x, y-1] if y > 0 else []
        surroundings["south"] = [x, y+1] if y < self.length-1 else []

        return surroundings

    def get_block_surroundings(self, x, y):
        """Returns neighbouring blocks (by edges, not vertices).

        Keyword arguments:
        x -- x-coordinate of block
        y -- y-coordinate of block
        """
        cartesian_surroundings = self.get_cartesian_surroundings(x, y)        

        surrounding_blocks = {"west": [], "east": [], "north": [], "south": []}

        for direction in cartesian_surroundings:
            if (cartesian_surroundings[direction]):
                block_x, block_y = cartesian_surroundings[direction]
                surrounding_blocks[direction] = self.maze[block_x][block_y]

        return surrounding_blocks

    def create_opening(self):
        """Creates first opening on one of the four walls of the maze.""" 
        if self.opening == 'west':
            for y in range(self.length):
                if (self.maze[1][y] == self.CELL):
                    self.maze[0][y] = self.CELL
                    break
        if self.opening == 'east':
            for y in range(self.length):
                if (self.maze[self.width-2][y] == self.CELL):
                    self.maze[self.width-1][y] = self.CELL
                    break
        if self.opening == 'north':
            for x in range(self.length):
                if (self.maze[x][1] == self.CELL):
                    self.maze[x][0] = self.CELL
                    break
        if self.opening == 'south':
            for x in range(self.length):
                if (self.maze[x][self.length-2] == self.CELL):
                    self.maze[x][self.length-1] = self.CELL
                    break

    def generate_maze(self, opening='west'):
        """Generates a maze using Prim's Random Maze Generation algorithm

        Keyword arguments:
        opening -- Opening to the maze
        """
        walls = [] # All coorindate pairs of walls in the maze

        # Initializing algorithm
        seed()
        STARTING_X = randint(1, self.width-2)
        STARTING_Y = randint(1, self.length-2)

        self.initialize() # Initializing maze with unvisited cells

        self.maze[STARTING_X][STARTING_Y] = self.CELL

        self.maze[STARTING_X-1][STARTING_Y] = self.WALL
        self.maze[STARTING_X][STARTING_Y-1] = self.WALL
        self.maze[STARTING_X][STARTING_Y+1] = self.WALL
        self.maze[STARTING_X+1][STARTING_Y] = self.WALL

        walls.append([STARTING_X-1, STARTING_Y])
        walls.append([STARTING_X, STARTING_Y-1])
        walls.append([STARTING_X, STARTING_Y+1])
        walls.append([STARTING_X+1, STARTING_Y])

        CARDINAL_COMPLEMENTS = {"west": "east", "east": "west", "north": "south", "south": "north"}

        # Turns walls (w) into a path block (c) when it divides an already visited path block and an unvisisted block (u)
        # only when the walls are surrounded by only one visited path block.
        while (walls):
            rand_wall = walls[randint(0, len(walls)-1)]
            wall_x, wall_y = rand_wall

            cartesian_surroundings = self.get_cartesian_surroundings(wall_x, wall_y)
            surrounding_blocks = self.get_block_surroundings(wall_x, wall_y)
            surrounding_cells = sum(block == self.CELL for block in surrounding_blocks.values())

            if (surrounding_cells == 1):
                for direction in surrounding_blocks:
                    if (surrounding_blocks[direction] == self.CELL and surrounding_blocks[CARDINAL_COMPLEMENTS[direction]] == self.UNVISITED_CELL):
                        self.maze[wall_x][wall_y] = self.CELL
                        break
                for direction in surrounding_blocks:
                    if (surrounding_blocks[direction] == self.UNVISITED_CELL):
                        new_x, new_y = cartesian_surroundings[direction]
                        self.maze[new_x][new_y] = self.WALL
                        walls.append([new_x, new_y])

            walls.remove([wall_x, wall_y])

        # Cleans up redundant blocks
        for col in range(self.width):
            for row in range(self.length):
                if (self.maze[col][row] == self.UNVISITED_CELL):
                    self.maze[col][row] = self.WALL

        self.create_opening()

    def generate_object(self):
        """Randomly generates an object position at a deadend in the maze."""
        deadends = []

        for col in range(self.width):
            for row in range(self.length):
                if (self.maze[col][row] == self.CELL):
                    surrounding_blocks = self.get_block_surroundings(col, row)
                    surrounding_cells = sum(block == self.WALL for block in surrounding_blocks.values())
                    if (surrounding_cells > 2):
                        deadends.append([col, row])
                        
        seed()
        rand_pos = randint(0, len(deadends)-1)
        obj_x, obj_y = deadends[rand_pos]
        self.maze[obj_x][obj_y] = self.OBJECT

if __name__ == "__main__":
    l = 20
    w = 25
    m = Maze(l, w, 'west')
    m.generate_maze()
    m.render()
    print()
    m.generate_object()
    m.render()