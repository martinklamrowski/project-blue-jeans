import time
import numpy as np


class Navigation:

    def __init__(self, h, w, startY=1, startX=0, orientation=1):

        self.directions = ((-1, 0), (0, 1), (1, 0), (0, -1))
        self.dirs = (0, 1, 2, 3, 0, 1, 2, 3)  # dir: 0 = up/North, 1 = right/East, 2 = down/South, 3 = left/West
        self.visited = np.zeros((h, w), np.bool_)
        self.map = np.zeros((h, w), np.uint8)
        """
        map legend:
        0 = U  = unexplored
        1 = W  = wall
        2 = E  = explored but not seen (by vision sensor)
        3 = S  = seen & explored
        """
        self.currY = 1  # startY
        self.currX = 0  # startX
        self.orientation = 1  # orientation
        self.exitY = 1  # startY
        self.exitX = 0  # startX
        self.h = h
        self.w = w
        # self.visited[self.currY,self.currX] = True

        # set what we know so far (i.e. the entrance & all other edge nodes are wall)
        self.map[0, :] = 1
        self.map[-1, :] = 1
        self.map[:, 0] = 1
        self.map[:, -1] = 1
        self.map[self.currY, self.currX] = 3

    def getnextPos(self, proxyData):

        # 1st update map based on new data
        self.__updateMap(proxyData)

        # 2nd wall flower maze exploring algorthm:
        if self.map[self.currY + self.directions[self.dirs[self.orientation] - 1][0],
                    self.currX + self.directions[self.dirs[self.orientation] - 1][
                        1]] != 1:  # check if left side is open
            self.currY += self.directions[self.dirs[self.orientation] - 1][0]
            self.currX += self.directions[self.dirs[self.orientation] - 1][1]
            self.map[self.currY, self.currX] = 3
            self.orientation = self.dirs[self.dirs[self.orientation] - 1]
            return ['L', 'C', 'F']
        elif self.map[self.currY + self.directions[self.dirs[self.orientation]][0],
                      self.currX + self.directions[self.dirs[self.orientation]][1]] != 1:  # check if front is open
            self.currY += self.directions[self.dirs[self.orientation]][0]
            self.currX += self.directions[self.dirs[self.orientation]][1]
            self.map[self.currY, self.currX] = 3
            return ['C', 'F']
        elif self.map[self.currY + self.directions[self.dirs[self.orientation + 1]][0],
                      self.currX + self.directions[self.dirs[self.orientation + 1]][1]] != 1:  # check if turn right
            self.currY += self.directions[self.dirs[self.orientation + 1]][0]
            self.currX += self.directions[self.dirs[self.orientation + 1]][1]
            self.orientation = self.dirs[self.orientation + 1]
            self.map[self.currY, self.currX] = 3
            return ['R', 'C', 'F']
        else:  # if nothing else to a u-turn
            self.currY += self.directions[self.dirs[self.orientation + 2]][0]
            self.currX += self.directions[self.dirs[self.orientation + 2]][1]
            self.orientation = self.dirs[self.orientation + 2]
            self.map[self.currY, self.currX] = 3
            return ['L', 'L', 'C', 'F']

    def goToExit(self):
        ys = [self.currY]
        xs = [self.currX]
        options = np.zeros((self.h, self.w), np.uint8)
        visited = np.zeros((self.h, self.w), np.bool_)
        visited[self.currY, self.currX] = True
        distance = 1
        while True:
            while len(ys) > 0:
                cur = (ys.pop(), xs.pop())
                for d, m in enumerate(self.__getMapOffsets()):
                    if (m[cur[0], cur[1]] > 1) and (
                            not visited[cur[0] + self.directions[d][0], cur[1] + self.directions[d][1]]):
                        options[cur[0] + self.directions[d][0], cur[1] + self.directions[d][1]] = distance
                        visited[cur[0] + self.directions[d][0], cur[1] + self.directions[d][1]] = True
                        if (cur[0] + self.directions[d][0] == self.exitY) and (
                                cur[1] + self.directions[d][1] == self.exitX):
                            return self.__convertToPathEXIT(options)
            yTemp, xTemp = np.where(options == distance)
            ys += yTemp.tolist()
            xs += xTemp.tolist()
            distance += 1

    def __updateMap(self, proxyData):
        # Left Sensor:
        if proxyData[0] == 0:
            if self.orientation == 1:
                self.map[self.currY - 1, self.currX] = 1
            elif self.orientation == 'W':
                self.map[self.currY + 1, self.currX] = 1
            elif self.orientation == 2:
                self.map[self.currY, self.currX + 1] = 1
            elif self.orientation == 0:
                self.map[self.currY, self.currX - 1] = 1
        elif proxyData[0] == 1:
            if self.orientation == 1:
                if self.map[self.currY - 1, self.currX] != 3:
                    self.map[self.currY - 1, self.currX] = 2
                self.map[self.currY - 2, self.currX] = 1
            elif self.orientation == 'W':
                if self.map[self.currY + 1, self.currX] != 3:
                    self.map[self.currY + 1, self.currX] = 2
                self.map[self.currY + 2, self.currX] = 1
            elif self.orientation == 2:
                if self.map[self.currY, self.currX + 1] != 3:
                    self.map[self.currY, self.currX + 1] = 2
                self.map[self.currY, self.currX + 2] = 1
            elif self.orientation == 0:
                if self.map[self.currY, self.currX - 1] != 3:
                    self.map[self.currY, self.currX - 1] = 2
                self.map[self.currY, self.currX - 2] = 1
        elif proxyData[0] == None:
            if self.orientation == 1:
                if self.map[self.currY - 1, self.currX] != 3:
                    self.map[self.currY - 1, self.currX] = 2
                if self.map[self.currY - 2, self.currX] != 3:
                    self.map[self.currY - 2, self.currX] = 2
            elif self.orientation == 'W':
                if self.map[self.currY + 1, self.currX] != 3:
                    self.map[self.currY + 1, self.currX] = 2
                if self.map[self.currY + 2, self.currX] != 3:
                    self.map[self.currY + 2, self.currX] = 2
            elif self.orientation == 2:
                if self.map[self.currY, self.currX + 1] != 3:
                    self.map[self.currY, self.currX + 1] = 2
                if self.map[self.currY, self.currX + 2] != 3:
                    self.map[self.currY, self.currX + 2] = 2
            elif self.orientation == 0:
                if self.map[self.currY, self.currX - 1] != 3:
                    self.map[self.currY, self.currX - 1] = 2
                if self.map[self.currY, self.currX - 2] != 3:
                    self.map[self.currY, self.currX - 2] = 2

        # Center Sensor: (it has vision so it will fully see the block)
        if proxyData[1] == 0:
            if self.orientation == 1:
                self.map[self.currY, self.currX + 1] = 1
            elif self.orientation == 3:
                self.map[self.currY, self.currX - 1] = 1
            elif self.orientation == 2:
                self.map[self.currY + 1, self.currX] = 1
            elif self.orientation == 0:
                self.map[self.currY - 1, self.currX] = 1
        elif proxyData[1] == 1:
            if self.orientation == 1:
                self.map[self.currY, self.currX + 1] = 3
                self.map[self.currY, self.currX + 2] = 1
            elif self.orientation == 3:
                self.map[self.currY, self.currX - 1] = 3
                self.map[self.currY, self.currX - 2] = 1
            elif self.orientation == 2:
                self.map[self.currY + 1, self.currX] = 3
                self.map[self.currY + 2, self.currX] = 1
            elif self.orientation == 0:
                self.map[self.currY - 1, self.currX] = 3
                self.map[self.currY - 2, self.currX] = 1
        elif proxyData[1] == None:
            if self.orientation == 1:
                self.map[self.currY, self.currX + 1] = 3
                if self.map[self.currY, self.currX + 2] != 3:
                    self.map[self.currY, self.currX + 2] = 2
            elif self.orientation == 3:
                self.map[self.currY, self.currX - 1] = 3
                if self.map[self.currY, self.currX - 2] != 3:
                    self.map[self.currY, self.currX - 2] = 2
            elif self.orientation == 2:
                self.map[self.currY + 1, self.currX] = 3
                if self.map[self.currY + 2, self.currX] != 3:
                    self.map[self.currY + 2, self.currX] = 2
            elif self.orientation == 0:
                self.map[self.currY - 1, self.currX] = 3
                if self.map[self.currY - 2, self.currX] != 3:
                    self.map[self.currY - 2, self.currX] = 2

                    # Right Sensor:
        if proxyData[2] == 0:
            if self.orientation == 1:
                self.map[self.currY + 1, self.currX] = 1
            elif self.orientation == 3:
                self.map[self.currY - 1, self.currX] = 1
            elif self.orientation == 2:
                self.map[self.currY, self.currX - 1] = 1
            elif self.orientation == 0:
                self.map[self.currY, self.currX + 1] = 1
        elif proxyData[2] == 1:
            if self.orientation == 1:
                if self.map[self.currY + 1, self.currX] != 3:
                    self.map[self.currY + 1, self.currX] = 2
                self.map[self.currY + 2, self.currX] = 1
            elif self.orientation == 3:
                if self.map[self.currY - 1, self.currX] != 3:
                    self.map[self.currY - 1, self.currX] = 2
                self.map[self.currY - 2, self.currX] = 1
            elif self.orientation == 2:
                if self.map[self.currY, self.currX - 1] != 3:
                    self.map[self.currY, self.currX - 1] = 2
                self.map[self.currY, self.currX - 2] = 1
            elif self.orientation == 0:
                if self.map[self.currY, self.currX + 1] != 3:
                    self.map[self.currY, self.currX + 1] = 2
                self.map[self.currY, self.currX + 2] = 1
        elif proxyData[2] == None:
            if self.orientation == 1:
                if self.map[self.currY + 1, self.currX] != 3:
                    self.map[self.currY + 1, self.currX] = 2
                if self.map[self.currY + 2, self.currX] != 3:
                    self.map[self.currY + 2, self.currX] = 2
            elif self.orientation == 3:
                if self.map[self.currY - 1, self.currX] != 3:
                    self.map[self.currY - 1, self.currX] = 2
                if self.map[self.currY - 2, self.currX] != 3:
                    self.map[self.currY - 2, self.currX] = 2
            elif self.orientation == 2:
                if self.map[self.currY, self.currX - 1] != 3:
                    self.map[self.currY, self.currX - 1] = 2
                if self.map[self.currY, self.currX - 2] != 3:
                    self.map[self.currY, self.currX - 2] = 2
            elif self.orientation == 0:
                if self.map[self.currY, self.currX + 1] != 3:
                    self.map[self.currY, self.currX + 1] = 2
                if self.map[self.currY, self.currX + 2] != 3:
                    self.map[self.currY, self.currX + 2] = 2

    def __convertToTurn(self, dir):  # dir: 0 = up/North, 1 = right/East, 2 = down/South, 3 = left/West
        if dir == 0:  # up
            if self.orientation == 1:
                self.orientation = 0
                return ['L']
            elif self.orientation == 3:
                self.orientation = 0
                return ['R']
            elif self.orientation == 2:
                self.orientation = 0
                return ['L', 'L']
            elif self.orientation == 0:
                return []
        elif dir == 1:  # right
            if self.orientation == 1:
                return []
            elif self.orientation == 3:
                self.orientation = 1
                return ['L', 'L']
            elif self.orientation == 2:
                self.orientation = 1
                return ['L']
            elif self.orientation == 0:
                self.orientation = 1
                return ['R']
        elif dir == 2:  # down
            if self.orientation == 1:
                self.orientation = 2
                return ['R']
            elif self.orientation == 3:
                self.orientation = 2
                return ['L']
            elif self.orientation == 2:
                return []
            elif self.orientation == 0:
                self.orientation = 2
                return ['L', 'L']
        elif dir == 3:  # left
            if self.orientation == 1:
                self.orientation = 3
                return ['L', 'L']
            elif self.orientation == 3:
                return []
            elif self.orientation == 2:
                self.orientation = 3
                return ['R']
            elif self.orientation == 0:
                self.orientation = 3
                return ['L']

    def __getMapOffsets(self):
        map = self.map.copy()
        # make extra maps (using same mem location) for more efficient comparisons between neighboring cells
        map_up = np.zeros((self.h + 1, self.w), np.uint8)  # create 4-neighbor connectivity comparision
        map_down = np.zeros((self.h + 1, self.w), np.uint8)
        map_right = np.zeros((self.h, self.w + 1), np.uint8)
        map_left = np.zeros((self.h, self.w + 1), np.uint8)
        map_up[1:, :] = map  # paste mask onto it, 1 shifted
        map_down[:-1, :] = map
        map_right[:, :-1] = map
        map_left[:, 1:] = map
        map_up = np.delete(map_up, -1, 0)  # delete the extra row/column
        map_down = np.delete(map_down, 0, 0)
        map_right = np.delete(map_right, 0, 1)
        map_left = np.delete(map_left, -1, 1)
        map_up[0, :] = 1  # set new cells (after the shift) to 1(walls) to eliminate false-positives
        map_down[-1, :] = 1
        map_right[:, -1] = 1
        map_left[:, 0] = 1
        # for dir, m in enumerate((map_up, map_down, map_right, map_left)):
        #     print(m)
        #     print(dir, currPos)
        return map_up, map_right, map_down, map_left

    def __convertToPathEXIT(self, options):
        options[options == 0] = 255
        options[self.currY, self.currX] = 0
        cur = (self.exitY, self.exitX)
        dirs = []
        while True:
            for dir in (0, 1, 2, 3):
                if options[cur[0], cur[1]] - 1 \
                        == options[cur[0] + self.directions[dir][0], cur[1] + self.directions[dir][1]]:
                    cur = (cur[0] + self.directions[dir][0], cur[1] + self.directions[dir][1])
                    dirs.append(dir)
                    break
            if (cur[0] == self.currY) and (cur[1] == self.currX):
                moves = []
                while len(dirs) > 0:
                    moves += self.__convertToTurn(self.dirs[dirs.pop() + 2]) + ['F']
                return moves
