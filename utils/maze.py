from random import seed, randint
from colorama import init, Fore

class Maze():
    def __init__(self, length, width) -> None:
        self.length = length
        self.width = width
        self.maze = []
        self.CELL = 'c'
        self.WALL = 'w'
        self.UNVISITED_CELL = 'u'

    def render(self):
        init()
        for row in range(self.length):
            for col in range(self.width):
                if (self.maze[col][row] == self.CELL):
                    print(Fore.WHITE, f'{self.maze[col][row]}' + ' ', end='')
                else:
                    print(Fore.RED, f'{self.maze[col][row]}' + ' ', end='')
            print()

    def initialize(self):
        for _ in range(self.width):
            row = []
            for _ in range(self.length):
                row.append(self.UNVISITED_CELL)
            self.maze.append(row)

    def get_cartesian_surroundings(self, x, y):
        surroundings = {}

        surroundings["west"] = [x-1, y] if x > 0 else []
        surroundings["east"] = [x+1, y] if x < self.width-1 else []
        surroundings["north"] = [x, y-1] if y > 0 else []
        surroundings["south"] = [x, y+1] if y < self.length-1 else []

        return surroundings

    def get_block_surroundings(self, x, y):
        cartesian_surroundings = self.get_cartesian_surroundings(x, y)        

        surrounding_blocks = {"west": [], "east": [], "north": [], "south": []}

        for direction in cartesian_surroundings:
            if (cartesian_surroundings[direction]):
                block_x, block_y = cartesian_surroundings[direction]
                surrounding_blocks[direction] = self.maze[block_x][block_y]

        return surrounding_blocks

    def create_opening(self, wall):
        if wall == 'west':
            for y in range(self.length):
                if self.maze[1][y] == self.CELL:
                    self.maze[0][y] = self.CELL
                    break
        if wall == 'east':
            for y in range(self.length):
                if self.maze[self.width-2][y] == self.CELL:
                    self.maze[self.width-1][y] = self.CELL
                    break
        if wall == 'north':
            for x in range(self.length):
                if self.maze[x][1] == self.CELL:
                    self.maze[x][0] = self.CELL
                    break
        if wall == 'south':
            for x in range(self.length):
                if self.maze[x][self.length-2] == self.CELL:
                    self.maze[x][self.length-1] = self.CELL
                    break

    def generate(self):
        walls = []

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
                for direction in surrounding_blocks:
                    if (surrounding_blocks[direction] == self.UNVISITED_CELL):
                        new_x, new_y = cartesian_surroundings[direction]
                        self.maze[new_x][new_y] = self.WALL
                        walls.append([new_x, new_y])

            walls.remove([wall_x, wall_y])

        for col in self.maze:
            for block in col:
                if block == self.UNVISITED_CELL:
                    block = self.WALL

        self.create_opening('west')

if __name__ == "__main__":
    l = 30
    w = 30
    m = Maze(l, w)
    m.generate()
    m.render()