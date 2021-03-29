import numpy as np
import sys

class RoboSTUB:
    def __init__(self, startY, startX, STUBmazeY=5, STUBmazeX=5):

        f = open("FINAL/maze_STUB.txt", 'r')
        self.map = np.zeros((STUBmazeY,STUBmazeX), np.uint8)
        for y,line in enumerate(f):
            data = line.rstrip('\n').split(' ')
            for x,d in enumerate(data):
                if d == 'w':    self.map[y][x] = 1
                if d == 'c':    self.map[y][x] = 0
                if d == 'O':    self.map[y][x] = 8
        f.close()
        self.currY = 1
        self.currX = 0
        self.orientation = 'E'

        self.visited = self.map.copy()

    def left(self):
        if   self.orientation == 'E':   self.orientation = 'N'
        elif self.orientation == 'W':   self.orientation = 'S'
        elif self.orientation == 'S':   self.orientation = 'E'
        elif self.orientation == 'N':   self.orientation = 'W'
    def right(self):
        if   self.orientation == 'E':   self.orientation = 'S'
        elif self.orientation == 'W':   self.orientation = 'N'
        elif self.orientation == 'S':   self.orientation = 'W'
        elif self.orientation == 'N':   self.orientation = 'E'
    def forward(self):
        if   self.orientation == 'E':   self.currX += 1
        elif self.orientation == 'W':   self.currX -= 1
        elif self.orientation == 'S':   self.currY += 1
        elif self.orientation == 'N':   self.currY -= 1

    def proxySensors(self):
        data = [None,None,None]
    # Left Sensor:
        if self.orientation == 'E':
            if   self.map[self.currY - 1, self.currX] == 1: data[0] = 0
            elif self.map[self.currY - 2, self.currX] == 1: data[0] = 1
        elif self.orientation == 'W':
            if   self.map[self.currY + 1, self.currX] == 1: data[0] = 0
            elif self.map[self.currY + 2, self.currX] == 1: data[0] = 1
        elif self.orientation == 'S':
            if   self.map[self.currY, self.currX + 1] == 1: data[0] = 0
            elif self.map[self.currY, self.currX + 2] == 1: data[0] = 1
        elif self.orientation == 'N':
            if   self.map[self.currY, self.currX - 1] == 1: data[0] = 0
            elif self.map[self.currY, self.currX - 2] == 1: data[0] = 1
    # Center Sensor: (it has vision so it will fully see the block)
        if self.orientation == 'E':
            if   self.map[self.currY, self.currX + 1] == 1: data[1] = 0
            elif self.map[self.currY, self.currX + 2] == 1: data[1] = 1
        elif self.orientation == 'W':
            if   self.map[self.currY, self.currX - 1] == 1: data[1] = 0
            elif self.map[self.currY, self.currX - 2] == 1: data[1] = 1
        elif self.orientation == 'S':
            if   self.map[self.currY + 1, self.currX] == 1: data[1] = 0
            elif self.map[self.currY + 2, self.currX] == 1: data[1] = 1
        elif self.orientation == 'N':
            if   self.map[self.currY - 1, self.currX] == 1: data[1] = 0
            elif self.map[self.currY - 2, self.currX] == 1: data[1] = 1
    # Right Sensor:
        if self.orientation == 'E':
            if   self.map[self.currY + 1, self.currX] == 1: data[2] = 0
            elif self.map[self.currY + 2, self.currX] == 1: data[2] = 1
        elif self.orientation == 'W':
            if   self.map[self.currY - 1, self.currX] == 1: data[2] = 0
            elif self.map[self.currY - 2, self.currX] == 1: data[2] = 1
        elif self.orientation == 'S':
            if   self.map[self.currY, self.currX - 1] == 1: data[2] = 0
            elif self.map[self.currY, self.currX - 2] == 1: data[2] = 1
        elif self.orientation == 'N':
            if   self.map[self.currY, self.currX + 1] == 1: data[2] = 0
            elif self.map[self.currY, self.currX + 2] == 1: data[2] = 1
        return data

    def pullVision(self):
        if self.orientation == 'E':
            self.visited[self.currY, self.currX + 1] = 5
            if self.map[self.currY, self.currX + 1] == 8:
                print('FOUND IT YAYAYYAYA')
                # sys.exit('WOOHOO BITCH')
                return True
            return False
        elif self.orientation == 'W':
            self.visited[self.currY, self.currX - 1] = 5
            if self.map[self.currY, self.currX - 1] == 8:
                print('FOUND IT YAYAYYAYA')
                # sys.exit('WOOHOO BITCH')
                return True
            return False
        elif self.orientation == 'S':
            self.visited[self.currY + 1, self.currX] = 5
            if self.map[self.currY + 1, self.currX] == 8:
                print('FOUND IT YAYAYYAYA')
                # sys.exit('WOOHOO BITCH')
                return True
            return False
        elif self.orientation == 'N':
            self.visited[self.currY - 1, self.currX] = 5
            if self.map[self.currY - 1, self.currX] == 8:
                print('FOUND IT YAYAYYAYA')
                # sys.exit('WOOHOO BITCH')
                return True
            return False