import numpy as np


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

    def move(self, move):
        if move == 'L':
            if   self.orientation == 'E':   self.orientation = 'N'
            elif self.orientation == 'W':   self.orientation = 'S'
            elif self.orientation == 'S':   self.orientation = 'E'
            elif self.orientation == 'N':   self.orientation = 'W'
        elif move == 'R':
            if   self.orientation == 'E':   self.orientation = 'S'
            elif self.orientation == 'W':   self.orientation = 'N'
            elif self.orientation == 'S':   self.orientation = 'W'
            elif self.orientation == 'N':   self.orientation = 'E'
        elif move == 'F':
            if   self.orientation == 'E':   self.currX += 1
            elif self.orientation == 'W':   self.currX -= 1
            elif self.orientation == 'S':   self.currY += 1
            elif self.orientation == 'N':   self.currY -= 1

    def proxySensors(self):
        data = [None,None,None]

        if self.orientation == 'E':
            offsetY = 1  # therefore left & right proxy sensors are looking up & down
            offsetX = 0  # since the center proxy is 90 degrees off we can just swap the x&y offsets bellow
        elif self.orientation == 'W':
            offsetY = -1
            offsetX = 0
        elif self.orientation == 'S':
            offsetY = 0
            offsetX = 1
        elif self.orientation == 'N':
            offsetY = 0
            offsetX = -1

    # Left Sensor:
        if self.map[self.currY + offsetY, self.currX - offsetX] == 1:  # if it sees a wall right next to it
            data[0] = 0
        elif self.map[self.currY + (offsetY * 2), self.currX - (offsetX * 2)] == 1:    # if it sees a wall 1 block away
            data[0] = 1

    # Center Sensor: (it has vision so it will fully see the block)
        if self.map[self.currY + offsetX, self.currX + offsetY] == 1:  # if it sees a wall right next to it
            data[1] = 0
        elif self.map[self.currY + (offsetX * 2), self.currX + (offsetY * 2)] == 1:    # if it sees a wall 1 block away
            data[1] = 1

    # Right Sensor:
        if self.map[self.currY + offsetY, self.currX + offsetX] == 1:  # if it sees a wall right next to it
            data[2] = 0
        elif self.map[self.currY + (offsetY * 2), self.currX + (offsetX * 2)] == 1:    # if it sees a wall 1 block away
            data[2] = 1

        return data