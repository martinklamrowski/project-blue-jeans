import time
import numpy as np
from skimage.segmentation import flood

class PathFinding:
    # def __init__(self):
    #     print('heyo from Path')


    def __init__(self, h, w, startY, startX, orientation='E'):
        print('heyo from Path!')

        self.directions = ((-1,0), (1,0), (0,1), (0,-1))
        # directions: 0=up, 1=down, 2=right, 3=left
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

        # set what we know so far (i.e. the entrance & all other edge nodes are wall)
        self.map[0,:] = 1; self.map[-1,:] = 1; self.map[:,0] = 1; self.map[:,-1] = 1
        self.map[self.currY, self.currX] = 3

    def getnextPos(self, proxyData):

        # print(proxyData, self.currY, self.currX)
        # print(self.map)
        self.__updateMap(proxyData)
        # print(self.map)

        # 1st check is at starting node
        if (self.currY == self.exitY) and (self.currX == self.exitX):   # if at start just move 1 forward
            if self.map[1,2] == 1:
                self.orientation = 'S'
                self.currX += 1
                return ['F', 'R']
            else:
                self.currX += 1
                return ['F']

        # 2nd check for num places to move
        opts = []
        for dir, m in enumerate(self.__getMapOffsets()):    # dir: 0=up, 1=down, 2=right, 3=left
            if m[self.currY,self.currX] != 1:
                opts.append(dir)

        if len(opts) == 0:
            x=1 # TODO: djikstra's to block before nearest 0 (& rotate to look at 0)

        elif len(opts) == 1:
            if self.map[self.currY+self.directions[opts[0]][0],self.currX+self.directions[opts[0]][1]] == 3:
                self.currY += self.directions[dir][0]
                self.currX += self.directions[dir][1]
                return ['F'] + self.__convertToTurn(opts[0])
            else:
                return self.__convertToTurn(opts[0])

        elif len(opts) > 1:
            ranks = []
            for dir in opts:  # these will be rank on which area has the least unknowns
                ranks.append(self.__rankedFlood(dir))
            # if len([index for index, element in enumerate(ranks) if min(ranks) == element]) > 1: # if a tie
            #     if self.orientation == 'E':
            choice = opts[ranks.index(min(ranks))]
            if self.map[self.currY+self.directions[choice][0],self.currX+self.directions[choice][1]] == 3:
                self.currY += self.directions[choice][0]
                self.currX += self.directions[choice][1]
                return ['F'] + self.__convertToTurn(choice)
            else:
                return self.__convertToTurn(choice)

        # TODO: using depth/flood 1st search if there is 2/E (or 3/S that connects to 2/Es) that are surrounded by walls -> then explore & return


    def goToExit(self):
        return ['R','L','F','R','L']


    def __updateMap(self, proxyData):
                                                        # Left Sensor:
        if proxyData[0] == 0:
            if self.orientation == 'E':
                self.map[self.currY - 1, self.currX] = 1
            elif self.orientation == 'W':
                self.map[self.currY + 1, self.currX] = 1
            elif self.orientation == 'S':
                self.map[self.currY, self.currX + 1] = 1
            elif self.orientation == 'N':
                self.map[self.currY, self.currX - 1] = 1
        elif proxyData[0] == 1:
            if self.orientation == 'E':
                if self.map[self.currY - 1, self.currX] != 3:
                    self.map[self.currY - 1, self.currX] = 2
                self.map[self.currY - 2, self.currX] = 1
            elif self.orientation == 'W':
                if self.map[self.currY + 1, self.currX] != 3:
                    self.map[self.currY + 1, self.currX] = 2
                self.map[self.currY + 2, self.currX] = 1
            elif self.orientation == 'S':
                if self.map[self.currY, self.currX + 1] != 3:
                    self.map[self.currY, self.currX + 1] = 2
                self.map[self.currY, self.currX + 2] = 1
            elif self.orientation == 'N':
                if self.map[self.currY, self.currX - 1] != 3:
                    self.map[self.currY, self.currX - 1] = 2
                self.map[self.currY, self.currX - 2] = 1
        elif proxyData[0] == None:
            if self.orientation == 'E':
                if self.map[self.currY - 1, self.currX] != 3:
                    self.map[self.currY - 1, self.currX] = 2
                if self.map[self.currY - 2, self.currX] != 3:
                    self.map[self.currY - 2, self.currX] = 2
            elif self.orientation == 'W':
                if self.map[self.currY + 1, self.currX] != 3:
                    self.map[self.currY + 1, self.currX] = 2
                if self.map[self.currY + 2, self.currX] != 3:
                    self.map[self.currY + 2, self.currX] = 2
            elif self.orientation == 'S':
                if self.map[self.currY, self.currX + 1] != 3:
                    self.map[self.currY, self.currX + 1] = 2
                if self.map[self.currY, self.currX + 2] != 3:
                    self.map[self.currY, self.currX + 2] = 2
            elif self.orientation == 'N':
                if self.map[self.currY, self.currX - 1] != 3:
                    self.map[self.currY, self.currX - 1] = 2
                if self.map[self.currY, self.currX - 2] != 3:
                    self.map[self.currY, self.currX - 2] = 2

                                                        # Center Sensor: (it has vision so it will fully see the block)
        if proxyData[1] == 0:
            if self.orientation == 'E':
                self.map[self.currY, self.currX + 1] = 1
            elif self.orientation == 'W':
                self.map[self.currY, self.currX - 1] = 1
            elif self.orientation == 'S':
                self.map[self.currY + 1, self.currX] = 1
            elif self.orientation == 'N':
                self.map[self.currY - 1, self.currX] = 1
        elif proxyData[1] == 1:
            if self.orientation == 'E':
                self.map[self.currY, self.currX + 1] = 3
                self.map[self.currY, self.currX + 2] = 1
            elif self.orientation == 'W':
                self.map[self.currY, self.currX - 1] = 3
                self.map[self.currY, self.currX - 2] = 1
            elif self.orientation == 'S':
                self.map[self.currY + 1, self.currX] = 3
                self.map[self.currY + 2, self.currX] = 1
            elif self.orientation == 'N':
                self.map[self.currY - 1, self.currX] = 3
                self.map[self.currY - 2, self.currX] = 1
        elif proxyData[1] == None:
            if self.orientation == 'E':
                self.map[self.currY, self.currX + 1] = 3
                if self.map[self.currY, self.currX + 2] != 3:
                    self.map[self.currY, self.currX + 2] = 2
            elif self.orientation == 'W':
                self.map[self.currY, self.currX - 1] = 3
                if self.map[self.currY, self.currX - 2] != 3:
                    self.map[self.currY, self.currX - 2] = 2
            elif self.orientation == 'S':
                self.map[self.currY + 1, self.currX] = 3
                if self.map[self.currY + 2, self.currX] != 3:
                    self.map[self.currY + 2, self.currX] = 2
            elif self.orientation == 'N':
                self.map[self.currY - 1, self.currX] = 3
                if self.map[self.currY - 2, self.currX] != 3:
                    self.map[self.currY - 2, self.currX] = 2

                                                        # Right Sensor:
        if proxyData[2] == 0:
            if self.orientation == 'E':
                self.map[self.currY + 1, self.currX] = 1
            elif self.orientation == 'W':
                self.map[self.currY - 1, self.currX] = 1
            elif self.orientation == 'S':
                self.map[self.currY, self.currX - 1] = 1
            elif self.orientation == 'N':
                self.map[self.currY, self.currX + 1] = 1
        elif proxyData[2] == 1:
            if self.orientation == 'E':
                if self.map[self.currY + 1, self.currX] != 3:
                    self.map[self.currY + 1, self.currX] = 2
                self.map[self.currY + 2, self.currX] = 1
            elif self.orientation == 'W':
                if self.map[self.currY - 1, self.currX] != 3:
                    self.map[self.currY - 1, self.currX] = 2
                self.map[self.currY - 2, self.currX] = 1
            elif self.orientation == 'S':
                if self.map[self.currY, self.currX - 1] != 3:
                    self.map[self.currY, self.currX - 1] = 2
                self.map[self.currY, self.currX - 2] = 1
            elif self.orientation == 'N':
                if self.map[self.currY, self.currX + 1] != 3:
                    self.map[self.currY, self.currX + 1] = 2
                self.map[self.currY, self.currX + 2] = 1
        elif proxyData[2] == None:
            if self.orientation == 'E':
                if self.map[self.currY + 1, self.currX] != 3:
                    self.map[self.currY + 1, self.currX] = 2
                if self.map[self.currY + 2, self.currX] != 3:
                    self.map[self.currY + 2, self.currX] = 2
            elif self.orientation == 'W':
                if self.map[self.currY - 1, self.currX] != 3:
                    self.map[self.currY - 1, self.currX] = 2
                if self.map[self.currY - 2, self.currX] != 3:
                    self.map[self.currY - 2, self.currX] = 2
            elif self.orientation == 'S':
                if self.map[self.currY, self.currX - 1] != 3:
                    self.map[self.currY, self.currX - 1] = 2
                if self.map[self.currY, self.currX - 2] != 3:
                    self.map[self.currY, self.currX - 2] = 2
            elif self.orientation == 'N':
                if self.map[self.currY, self.currX + 1] != 3:
                    self.map[self.currY, self.currX + 1] = 2
                if self.map[self.currY, self.currX + 2] != 3:
                    self.map[self.currY, self.currX + 2] = 2

    def __convertToTurn(self, dir):
        if dir == 0: # up
            if   self.orientation == 'E':
                self.orientation = 'N'
                return ['L']
            elif self.orientation == 'W':
                self.orientation = 'N'
                return ['R']
            elif self.orientation == 'S':
                self.orientation = 'N'
                return ['L','L']
            elif self.orientation == 'N':
                return []
        elif   dir == 1: # down
            if   self.orientation == 'E':
                self.orientation = 'S'
                return ['R']
            elif self.orientation == 'W':
                self.orientation = 'S'
                return ['L']
            elif self.orientation == 'S':
                return []
            elif self.orientation == 'N':
                self.orientation = 'S'
                return ['L','L']
        elif dir == 2: # right
            if   self.orientation == 'E':
                return []
            elif self.orientation == 'W':
                self.orientation = 'E'
                return ['L','L']
            elif self.orientation == 'S':
                self.orientation = 'E'
                return ['L']
            elif self.orientation == 'N':
                self.orientation = 'E'
                return ['R']
        elif dir == 3: # left
            if   self.orientation == 'E':
                self.orientation = 'W'
                return ['L','L']
            elif self.orientation == 'W':
                return []
            elif self.orientation == 'S':
                self.orientation = 'W'
                return ['R']
            elif self.orientation == 'N':
                self.orientation = 'W'
                return ['L']

    def __getMapOffsets(self, currPos=None):
        map = self.map.copy()
        if currPos != None:
            map[currPos[0],currPos[1]] = 5
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
        return (map_up, map_down, map_right, map_left)

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
