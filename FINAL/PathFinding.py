import time
import numpy as np


class PathFinding:
    # def __init__(self):
    #     print('heyo from Path')

    def __init__(self, h, w, startY, startX, orientation='E'):
        print('heyo from Path!')

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
        self.__updateMap(proxyData)

        # 1st check is at starting node
        if (self.currY == self.exitY) and (self.currX == self.exitX): # if at start just move 1 forward
            self.currX += 1                                    # (it just a simple solution to negate all indexing errs)
            return ['F']

        # 2nd check for num places to move
        opts = []
        for dir, m in enumerate(self.__getMapOffsets()):    # dir: 0=up, 1=down, 2=right, 3=left
            if m[self.currY,self.currX] != 1:
                opts.append(dir)

        if len(opts) == 0:
            x=1 # TODO: djikstra's to block before nearest 0 (& rotate to look at 0)
        if len(opts) > 1:
            ranks = []
            options = []
            for n, o in enumerate(opts): # these will be rank on which area has the most unknowns
                yy = self.currY
                xx = self.currX
                stack = []
                options.append([])
                for dir, m in enumerate(self.__getMapOffsets([self.currY,self.currX])): # set curr position to 1(wall) so we dont scan it
                    if m[yy,xx] != 1:
                        if dir == 0:
                            stack.append([yy-1,xx])
                            options[n].append()
                        elif dir == 1:
                            stack.append([yy+1,xx])
                        elif dir == 2:
                            stack.append([yy,xx+1])
                        elif dir == 3:
                            stack.append([yy,xx-1])


        # print(self.__convertToTurn(opts[0]))
        # print(opts)
        # time.sleep(10)
            # return self.__convertToMove(opts[0])    # TODO : if 2/E we should look 1st then move


        # TODO: using depth/flood 1st search if there is 2/E (or 3/S that connects to 2/Es) that are surrounded by walls -> then explore & return



    def goToExit(self):
        return ['R','L','F','R','L']


    def __updateMap(self, proxyData):

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
        if proxyData[0] == 0:
            self.map[self.currY + offsetY, self.currX - offsetX] = 1  # if it sees a wall right next to it
        elif proxyData[0] == 1:
            self.map[self.currY + offsetY, self.currX - offsetX] = 2  # if it sees a wall 1 block away
            self.map[self.currY + (offsetY * 2), self.currX - (offsetX * 2)] = 1
        elif proxyData[0] == None:
            self.map[self.currY + offsetY, self.currX - offsetX] = 2  # if is sees noting
            self.map[self.currY + (offsetY * 2), self.currX - (offsetX * 2)] = 2
        # Center Sensor: (it has vision so it will fully see the block)
        if proxyData[1] == 0:
            self.map[self.currY + offsetX, self.currX + offsetY] = 1  # if it sees a wall right next to it
        elif proxyData[1] == 1:
            self.map[self.currY + offsetX, self.currX + offsetY] = 3  # if it sees a wall 1 block away
            self.map[self.currY + (offsetX * 2), self.currX + (offsetY * 2)] = 1
        elif proxyData[1] == None:
            self.map[self.currY + offsetX, self.currX - offsetY] = 3  # if is sees noting
            self.map[self.currY + (offsetX * 2), self.currX - (offsetY * 2)] = 2
        # Right Sensor:
        if proxyData[2] == 0:
            self.map[self.currY + offsetY, self.currX + offsetX] = 1  # if it sees a wall right next to it
        elif proxyData[2] == 1:
            self.map[self.currY + offsetY, self.currX + offsetX] = 2  # if it sees a wall 1 block away
            self.map[self.currY + (offsetY * 2), self.currX + (offsetX * 2)] = 1
        elif proxyData[2] == None:
            self.map[self.currY + offsetY, self.currX + offsetX] = 2  # if is sees noting
            self.map[self.currY + (offsetY * 2), self.currX + (offsetX * 2)] = 2

    def __convertToTurn(self, dir):
        if   dir == [1,0]: # down
            if   self.orientation == 'E':   return ['R']
            elif self.orientation == 'W':   return ['L']
            elif self.orientation == 'S':   return []
            elif self.orientation == 'N':   return ['L','L']
        elif dir == [-1,0]: # up
            if   self.orientation == 'E':   return ['L']
            elif self.orientation == 'W':   return ['R']
            elif self.orientation == 'S':   return ['L','L']
            elif self.orientation == 'N':   return []
        elif dir == [0,1]: # right
            if   self.orientation == 'E':   return ['L','L']
            elif self.orientation == 'W':   return []
            elif self.orientation == 'S':   return ['L']
            elif self.orientation == 'N':   return ['R']
        elif dir == [0,-1]: # left
            if   self.orientation == 'E':   return []
            elif self.orientation == 'W':   return ['L','L']
            elif self.orientation == 'S':   return ['R']
            elif self.orientation == 'N':   return ['L']

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
        for dir, m in enumerate((map_up, map_down, map_right, map_left)):
            print(m)
            print(dir, currPos)
        return (map_up, map_down, map_right, map_left)