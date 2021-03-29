import time
import numpy as np
from skimage.segmentation import flood

class PathFinding:
    # def __init__(self):
    #     print('heyo from Path')
    #     self.map = [['exit']]
    #     self.visited = [[True]]
    #     self.orientation = 'E'


    def __init__(self, h, w, startY, startX, orientation=1):
        print('heyo from Path!')

        self.directions = ((-1,0), (0,1), (1,0), (0,-1))
        self.dirs = (0,1,2,3,0,1,2,3)                     # dir: 0 = up/North, 1 = right/East, 2 = down/South, 3 = left/West
        self.visited = np.zeros((h, w), np.bool_)
        self.map = np.zeros((h,w), np.uint8)
        """
        map legend:
        0 = U  = unexplored
        1 = W  = wall
        2 = E  = explored but not seen (by vision sensor)
        3 = S  = seen & explored
        """
        self.currY = startY
        self.currX = startX
        self.orientation = orientation
        self.exitY = startY
        self.exitX = startX
        self.h = h
        self.w = w
        self.visited[self.currY,self.currX] = True

        # set what we know so far (i.e. the entrance & all other edge nodes are wall)
        self.map[0,:] = 1; self.map[-1,:] = 1; self.map[:,0] = 1; self.map[:,-1] = 1
        self.map[self.currY, self.currX] = 3

    def getnextPos(self, proxyData):

        # 0th update map based on new data
        self.__updateMap(proxyData)
        # 1st check if pants have been found
        if len(proxyData) > 3:
            xxx=1   # TODO: djikstra's

        # 2nd wall flower maze exploring algorthm:
        if self.map[self.currY+self.directions[self.dirs[self.orientation]-1][0],
                    self.currX+self.directions[self.dirs[self.orientation]-1][1]] != 1:     # check if left side is open
            self.currY += self.directions[self.dirs[self.orientation]-1][0]
            self.currX += self.directions[self.dirs[self.orientation]-1][1]
            self.orientation = self.dirs[self.dirs[self.orientation]-1]
            return ['L','F']
        elif self.map[self.currY+self.directions[self.dirs[self.orientation]][0],
                    self.currX+self.directions[self.dirs[self.orientation]][1]] != 1:           # check if front is open
            self.currY += self.directions[self.dirs[self.orientation]][0]
            self.currX += self.directions[self.dirs[self.orientation]][1]
            return ['F']
        elif self.map[self.currY+self.directions[self.dirs[self.orientation+1]][0],
                    self.currX+self.directions[self.dirs[self.orientation+1]][1]] != 1:            # check if turn right
            self.currY += self.directions[self.dirs[self.orientation+1]][0]
            self.currX += self.directions[self.dirs[self.orientation+1]][1]
            self.orientation = self.dirs[self.orientation+1]
            return ['R','F']
        else:                                                                              # if nothing else to a u-turn
            self.currY += self.directions[self.dirs[self.orientation + 2]][0]
            self.currX += self.directions[self.dirs[self.orientation + 2]][1]
            self.orientation = self.dirs[self.orientation + 2]
            return ['L','L','F']



        # if self.__checkDirection()[self.currY,self.currX]

        # opts = []
        # for dir, m in enumerate(self.__getMapOffsets()):    # dir: 0 = up/N, 1 = right/E, 2 = down/S, 3 = left/S
        #     if m[self.currY,self.currX] != 1:
        #         opts.append(dir)
        # xxx=1



        # 1st check is at starting node
        # if (self.currY == self.exitY) and (self.currX == self.exitX):   # if at start just move 1 forward
        #     if self.map[1,2] == 1:
        #         self.orientation = 'S'
        #         self.currX += 1
        #         self.visited[self.currY, self.currX] = True
        #         return ['F', 'R']
        #     else:
        #         self.currX += 1
        #         self.visited[self.currY, self.currX] = True
        #         return ['F']
        #
        # # 2nd check for num places to move
        # opts = []
        # for dir, m in enumerate(self.__getMapOffsets()):    # dir: 0=up, 1=down, 2=right, 3=left
        #     if m[self.currY,self.currX] != 1:
        #         opts.append(dir)
        #
        # if len(opts) == 0:
        #     x=1 # TODO: djikstra's to block before nearest 0 (& rotate to look at 0)
        #
        # elif len(opts) == 1:
        #     if self.map[self.currY+self.directions[opts[0]][0],self.currX+self.directions[opts[0]][1]] == 3:
        #         self.currY += self.directions[dir][0]
        #         self.currX += self.directions[dir][1]
        #         self.visited[self.currY, self.currX] = True
        #         return ['F'] + self.__convertToTurn(opts[0])
        #     else:
        #         return self.__convertToTurn(opts[0])
        #
        # elif len(opts) > 1:
        #     ranks = []
        #     for dir in opts:  # these will be rank on which area has the least unknowns
        #         ranks.append(self.__rankedFlood(dir))
        #     # if len([index for index, element in enumerate(ranks) if min(ranks) == element]) > 1: # if a tie
        #     #     if self.orientation == 'E':
        #     choice = opts[ranks.index(min(ranks))]
        #     if self.map[self.currY+self.directions[choice][0],self.currX+self.directions[choice][1]] == 3:
        #         self.currY += self.directions[choice][0]
        #         self.currX += self.directions[choice][1]
        #         self.visited[self.currY, self.currX] = True
        #         return ['F'] + self.__convertToTurn(choice)
        #     else:
        #         return self.__convertToTurn(choice)

        # TODO: using depth/flood 1st search if there is 2/E (or 3/S that connects to 2/Es) that are surrounded by walls -> then explore & return


    def goToExit(self):
        return ['R','L','F','R','L']


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

    def __convertToTurn(self, dir):         # dir: 0 = up/North, 1 = right/East, 2 = down/South, 3 = left/West
        if dir == 0: # up
            if   self.orientation == 1:
                self.orientation = 0
                return ['L']
            elif self.orientation == 3:
                self.orientation = 0
                return ['R']
            elif self.orientation == 2:
                self.orientation = 0
                return ['L','L']
            elif self.orientation == 0:
                return []
        elif dir == 1: # right
            if   self.orientation == 1:
                return []
            elif self.orientation == 3:
                self.orientation = 1
                return ['L','L']
            elif self.orientation == 2:
                self.orientation = 1
                return ['L']
            elif self.orientation == 0:
                self.orientation = 1
                return ['R']
        elif   dir == 2: # down
            if   self.orientation == 1:
                self.orientation = 2
                return ['R']
            elif self.orientation == 3:
                self.orientation = 2
                return ['L']
            elif self.orientation == 2:
                return []
            elif self.orientation == 0:
                self.orientation = 2
                return ['L','L']
        elif dir == 3: # left
            if   self.orientation == 1:
                self.orientation = 3
                return ['L','L']
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
        map[self.visited==True] = 1
        # make extra maps (using same mem location) for more efficient comparisons between neighboring cells
        map_up = np.zeros((self.h + 1, self.w), np.uint8)  # create 4-neighbor connectivity comparision
        map_down = np.zeros((self.h + 1, self.w), np.uint8)
        map_right = np.zeros((self.h, self.w + 1), np.uint8)
        map_left = np.zeros((self.h, self.w + 1), np.uint8)
        map_up[1:, :] = map              # paste mask onto it, 1 shifted
        map_down[:-1, :] = map
        map_right[:, :-1] = map
        map_left[:, 1:] = map
        map_up = np.delete(map_up, -1, 0)     # delete the extra row/column
        map_down = np.delete(map_down, 0, 0)
        map_right = np.delete(map_right, 0, 1)
        map_left = np.delete(map_left, -1, 1)
        map_up[0,:] = 1                       # set new cells (after the shift) to 1(walls) to eliminate false-positives
        map_down[-1,:] = 1
        map_right[:,-1] = 1
        map_left[:,0] = 1
        # for dir, m in enumerate((map_up, map_down, map_right, map_left)):
        #     print(m)
        #     print(dir, currPos)
        return (map_up, map_right, map_down, map_left)

    def __rankedFlood(self, seedDir):
        mask = np.zeros((self.h,self.w), np.bool_)
        mask[self.map!=1] = True # set all walls to background
        mask[self.currY,self.currX] = False # set curr position to background, so the flood dont initially overlap everytime

        area = flood(mask,(self.currY + self.directions[seedDir][0],self.currX + self.directions[seedDir][1]), connectivity=1) #1=excludes diagonals

        points = np.zeros((self.h,self.w), np.uint8)
        points[self.map==0] = 1
        points[self.map==2] = 2

        sum = np.sum(points[area])
        if sum == 0:
            return self.h * self.w * 2   # if only 3(seen&fully explored) blocks, this will garuntee it wont go this way
        else:
            return sum

    def __checkDirection(self):
        map = self.map.copy()
        map[self.visited==True] = 1

        # make extra maps (using same mem location) for more efficient comparisons between neighboring cells
        map_up = np.zeros((self.h + 1, self.w), np.uint8)  # create 4-neighbor connectivity comparision
        map_down = np.zeros((self.h + 1, self.w), np.uint8)
        map_right = np.zeros((self.h, self.w + 1), np.uint8)
        map_left = np.zeros((self.h, self.w + 1), np.uint8)
        map_up[1:, :] = map              # paste mask onto it, 1 shifted
        map_down[:-1, :] = map
        map_right[:, :-1] = map
        map_left[:, 1:] = map
        map_up = np.delete(map_up, -1, 0)     # delete the extra row/column
        map_down = np.delete(map_down, 0, 0)
        map_right = np.delete(map_right, 0, 1)
        map_left = np.delete(map_left, -1, 1)
        map_up[0,:] = 1                       # set new cells (after the shift) to 1(walls) to eliminate false-positives
        map_down[-1,:] = 1
        map_right[:,-1] = 1
        map_left[:,0] = 1
        # for dir, m in enumerate((map_up, map_down, map_right, map_left)):
        #     print(m)
        #     print(dir, currPos)
        return (map_up, map_right, map_down, map_left)

if __name__ == '__main__':
    p = PathFinding()
