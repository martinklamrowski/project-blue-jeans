import sys
import numpy as np

# required for non-truncated map print
np.set_printoptions(threshold=sys.maxsize)


class Navigation:

    def __init__(self, h, w, startY=1, startX=0, orientation=1):
        self.directions = ((-1, 0), (0, 1), (1, 0), (0, -1))
        self.dirs = (0, 1, 2, 3, 0, 1, 2, 3)
        # dir: 0 = up/North, 1 = right/East, 2 = down/South, 3 = left/West
        self.visited = np.zeros((h, w), np.bool_)
        self.map = np.zeros((h, w), np.uint8)
        """
        map legend:
        0 = unexplored
        1 = wall
        2 = explored but not seen (by vision sensor)
        3 = seen & explored
        """
        self.currY = 1  # startY
        self.currX = 0  # startX
        self.orientation = 1  # orientation
        self.exitY = 1  # startY
        self.exitX = 0  # startX
        self.h = h
        self.w = w

        # set what we know so far (i.e. the entrance & all other edge nodes are wall)
        self.map[0, :] = 1
        self.map[-1, :] = 1
        self.map[:, 0] = 1
        self.map[:, -1] = 1
        self.map[self.currY, self.currX] = 3

    def display(self):
        """
        Displays the map as the Robo updates it.
        :return: None
        """
        map_show = self.map.copy()
        map_show[self.currY, self.currX] = 4        
        print(map_show)

    def get_next_pos(self, proxy_data):
        """
        pases proxy_data to update map then
        uses wallflower algorithm to determine next set of moves
        :param:     proxy_data = data from surrounding blocks (defined in greater detail on line 141 for __update_map() )
        :return:    moves = list of next moves for robot
                                R = pivot right
                                L = pivot left
                                F = move forward
                                C = capture vision sensor & check for pants
        """
        # 1st update map based on new data
        self.__update_map(proxy_data)

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

    def go_to_exit(self):
        """
        uses depth first search to find shortest route to exit
        :return:    moves = list of next moves for robot
                                R = pivot right
                                L = pivot left
                                F = move forward
        """
        ys = [self.currY]
        xs = [self.currX]
        options = np.zeros((self.h, self.w), np.uint8)
        visited = np.zeros((self.h, self.w), np.bool_)
        visited[self.currY, self.currX] = True
        distance = 1
        while True:
            while len(ys) > 0:
                cur = (ys.pop(), xs.pop())
                for d, m in enumerate(self.__get_map_offsets()):
                    if (m[cur[0], cur[1]] > 1) and (
                            not visited[cur[0] + self.directions[d][0], cur[1] + self.directions[d][1]]):
                        options[cur[0] + self.directions[d][0], cur[1] + self.directions[d][1]] = distance
                        visited[cur[0] + self.directions[d][0], cur[1] + self.directions[d][1]] = True
                        if (cur[0] + self.directions[d][0] == self.exitY) and (
                                cur[1] + self.directions[d][1] == self.exitX):
                            return self.__convert_to_path_exit(options)
            yTemp, xTemp = np.where(options == distance)
            ys += yTemp.tolist()
            xs += xTemp.tolist()
            distance += 1

    def __update_map(self, proxy_data):
        """
        updates map based on proxy sensors
        :param:     proxy_data = data from surrounding blocks [Left, Center, Right]
                                0 = wall right next to robot
                                None = no wall next to robot
        :return:    moves = list of next moves for robot
                                R = pivot right
                                L = pivot left
                                F = move forward
                                C = capture vision sensor & check for pants
        """
        # Left Sensor:
        if proxy_data[0] == 0:
            if self.orientation == 1:
                self.map[self.currY - 1, self.currX] = 1
            elif self.orientation == 3:
                self.map[self.currY + 1, self.currX] = 1
            elif self.orientation == 2:
                self.map[self.currY, self.currX + 1] = 1
            elif self.orientation == 0:
                self.map[self.currY, self.currX - 1] = 1
        elif proxy_data[0] is None:
            if self.orientation == 1:
                if self.map[self.currY - 1, self.currX] != 3:
                    self.map[self.currY - 1, self.currX] = 2
            elif self.orientation == 3:
                if self.map[self.currY + 1, self.currX] != 3:
                    self.map[self.currY + 1, self.currX] = 2
            elif self.orientation == 2:
                if self.map[self.currY, self.currX + 1] != 3:
                    self.map[self.currY, self.currX + 1] = 2
            elif self.orientation == 0:
                if self.map[self.currY, self.currX - 1] != 3:
                    self.map[self.currY, self.currX - 1] = 2

        # Center Sensor: (it has vision so it will fully see the block)
        if proxy_data[1] == 0:
            if self.orientation == 1:
                self.map[self.currY, self.currX + 1] = 1
            elif self.orientation == 3:
                self.map[self.currY, self.currX - 1] = 1
            elif self.orientation == 2:
                self.map[self.currY + 1, self.currX] = 1
            elif self.orientation == 0:
                self.map[self.currY - 1, self.currX] = 1
        elif proxy_data[1] is None:
            if self.orientation == 1:
                self.map[self.currY, self.currX + 1] = 3
            elif self.orientation == 3:
                self.map[self.currY, self.currX - 1] = 3
            elif self.orientation == 2:
                self.map[self.currY + 1, self.currX] = 3
            elif self.orientation == 0:
                self.map[self.currY - 1, self.currX] = 3

        # Right Sensor:
        if proxy_data[2] == 0:
            if self.orientation == 1:
                self.map[self.currY + 1, self.currX] = 1
            elif self.orientation == 3:
                self.map[self.currY - 1, self.currX] = 1
            elif self.orientation == 2:
                self.map[self.currY, self.currX - 1] = 1
            elif self.orientation == 0:
                self.map[self.currY, self.currX + 1] = 1
        elif proxy_data[2] is None:
            if self.orientation == 1:
                if self.map[self.currY + 1, self.currX] != 3:
                    self.map[self.currY + 1, self.currX] = 2
            elif self.orientation == 3:
                if self.map[self.currY - 1, self.currX] != 3:
                    self.map[self.currY - 1, self.currX] = 2
            elif self.orientation == 2:
                if self.map[self.currY, self.currX - 1] != 3:
                    self.map[self.currY, self.currX - 1] = 2
            elif self.orientation == 0:
                if self.map[self.currY, self.currX + 1] != 3:
                    self.map[self.currY, self.currX + 1] = 2

    def __convert_to_turn(self, dir):
        """
        converts direction of next block to go to, into robot moves. based on robots current orientation
        :param:     dir = direction of next block to move to
                                0 = up/North
                                1 = right/East
                                2 = down/South
                                3 = left/West
        :return:    move(s) = list of next move or moves for robot
                                R = pivot right
                                L = pivot left
                                F = move forward
                                C = capture vision sensor & check for pants
        """
        if dir == 0:                                # up
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
        elif dir == 1:                              # right
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
        elif dir == 2:                              # down
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
        elif dir == 3:                              # left
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

    def __get_map_offsets(self):
        """
        return extra maps (using same mem location) for more efficient comparisons between neighboring cells
        :param:     dir = direction of next block to move to
                                0 = up/North
                                1 = right/East
                                2 = down/South
                                3 = left/West
        :return:    mapOffsets = tuple of matrices shifted by 1 to compare neighboring blocks easily
                                        (shifted up, shifted right, shifted down, shifted left)
        """
        map = self.map.copy()
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
        return map_up, map_right, map_down, map_left

    def __convert_to_path_exit(self, options):
        """
        once depth first search has found the exit, this will convert the matrix to a single path
                                                                      & return as robot functions
        :param:     options = matrix showing results of the dapth 1st searh
        :return:    moves = list of next moves for robot
                                R = pivot right
                                L = pivot left
                                F = move forward
                                C = capture vision sensor & check for pants
        """
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
                    moves += self.__convert_to_turn(self.dirs[dirs.pop() + 2]) + ['F']
                return moves
