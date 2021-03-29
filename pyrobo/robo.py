import math
import keyboard

import utils.constants as consts
import utils.vec as vec
from utils.navigation import Navigation


class Robo(object):
    def __init__(self, boundary=None, testing=True, manual=False):
        print("Well hello there.")

        # if you don't want to use Coppelia
        self.testing = testing

        # the Robo's perceived position and orientation within the maze_map,
        # starts at 0, 0 (top left corner)
        self.pos_i = 0
        self.pos_j = 0
        self.vel_x = 0
        self.vel_y = 0
        self.orientation = consts.EAST  # TODO : Change this to deg or rad.
        self.mode = consts.EXPLORATION_MODE

        # interface with Coppelia
        # TODO: Remove testing argument.
        if boundary is None and not testing:
            raise TypeError("""
                Sorry, Robo needs a Boundary object; you haven't supplied one.\n
                __init__(self, boundary) : boundary == None
            """)
        self.boundary = boundary
        self.MANUAL = manual

    def run(self):
        if not self.MANUAL:
            nav = Navigation(18,18) # TODO: input (h,w) as variables from maze
            # Exploring:
            while True:
                # collect data:
                proxyData = self.boundary_get() # TODO:
                moves = nav.getnextPos(proxyData)
                # move:
                for m in moves:
                    foundPants = self.move(m)
                    if foundPants:
                        break
                if foundPants:
                    break

            moves = nav.goToExit()
            for m in moves:
                self.move(m)

            self.dance()
            self.boundary.closeConnection()

        else:
            while True:
                if keyboard.is_pressed("u"):
                    self.__raise_arm_step(consts.LEFT_ARM)
                elif keyboard.is_pressed("j"):
                    self.__lower_arm_step(consts.LEFT_ARM)
                if keyboard.is_pressed("i"):
                    self.__raise_arm_step(consts.RIGHT_ARM)
                elif keyboard.is_pressed("k"):
                    self.__lower_arm_step(consts.RIGHT_ARM)

                if keyboard.is_pressed("w"):
                    if keyboard.is_pressed("a"):
                        self.__accelerate_forward(turn="left")
                    elif keyboard.is_pressed("d"):
                        self.__accelerate_forward(turn="right")
                    else:
                        self.__accelerate_forward(turn=None)
                elif keyboard.is_pressed("s"):
                    if keyboard.is_pressed("a"):
                        self.__accelerate_backward(turn="left")
                    elif keyboard.is_pressed("d"):
                        self.__accelerate_backward(turn="right")
                    else:
                        self.__accelerate_backward(turn=None)
                elif keyboard.is_pressed("e"):
                    self.__turn_right_on_spot(consts.EAST)
                elif keyboard.is_pressed("q"):
                    self.__turn_right_on_spot(consts.WEST)
                elif keyboard.is_pressed("c"):
                    self.__turn_right_on_spot(consts.NORTH)
                elif keyboard.is_pressed("q"):
                    self.__turn_right_on_spot(consts.WEST)
                else:
                    self.__decelerate()

                print("{} X | {} Y".format(self.__get_vel_x(), self.__get_vel_y()))

    def move(self, move):
        if move == 'L':
            self.__turn_right_on_spot(consts.WEST)
            self.__turn_right_on_spot(consts.WEST)
            self.__turn_right_on_spot(consts.WEST)
            print('left pivot')
        elif move == 'R':
            self.__turn_right_on_spot(consts.WEST)
            print('right pivot')
        elif move == 'F':
            self.__accelerate_forward(turn=None)
            print('move 1 forward')
        elif move == 'C':
            if self.boundary.get_vision():
                self.pickUp()
                return True
        return False

    def pickUp(self):
        if self.STUB: self.roboSTUB.forward() # since vision sensor only detects 1 in front, pickup() will need to move forward to pick up the pants
        print('woohoo!')

    def dance(self):
        for AYO_MUTHA_FUCKA in range(int(6.9)):
            print('DANCINGGGG')

    def __move_to_next(self, testing_map=None):
        """
        This will be the method to compute the next move for the Robo based on its current square.
        TODO: Need sensor info from Boundary here, will emulate for now. Remove testing_map
              eventually. Need better algorithm too, right now it just follows the left wall.

        :return: None
        """
        if self.testing:
            if testing_map is None:
                raise TypeError("""
                    You enabled testing but haven't supplied a testing_map.\n
                    move_to_next(self, testing_map) : testing_map == None
                """)
            if self.mode == consts.EXPLORATION_MODE:

                current_node = testing_map.get_map_node_at_pos(self.pos_j, self.pos_i)
                self.maze_map.set_map_node(current_node) # TODO
                self.__update_adjacent_map_nodes(current_node)

                # is there a wall to the left of the Robo in its current square?
                if current_node.walls[self.__get_left_cardinality()]:
                    print("Found wall to the left.")
                    if current_node.walls[self.orientation]:
                        print("Found wall in front.")
                        if current_node.walls[self.__get_right_cardinality()]:
                            print("Found wall to the right.")
                            print("Walls all around, going back.")
                            self.__update_pos(self.__get_rear_cardinality())
                        else:
                            print("Walls front and left, going right.")
                            self.__update_pos(self.__get_right_cardinality())
                    else:
                        print("Wall to the left only, moving forwards.")
                        self.__update_pos(self.orientation)
                else:
                    print("No left wall, going left.")
                    self.__update_pos(self.__get_left_cardinality())

    # TODO: Merge these cardinality methods.
    def __get_left_cardinality(self):
        """
        Returns the cardinal point to the Robo's left. I.e., the true cardinal point
        of the Robo's West.

        :return: int -> representing a cardinal point from constants.py
        """
        if self.orientation == consts.NORTH:
            cardinal_point = consts.WEST
        else:
            cardinal_point = self.orientation - 1

        return cardinal_point

    def __get_right_cardinality(self):
        """
        Returns the cardinal point to the Robo's right. I.e., the true cardinal point
        of the Robo's East.

        :return: int -> representing a cardinal point from constants.py
        """
        if self.orientation == consts.WEST:
            cardinal_point = consts.NORTH
        else:
            cardinal_point = self.orientation + 1

        return cardinal_point

    def __get_rear_cardinality(self):
        """
        Returns the cardinal point to the Robo's rear. I.e., the true cardinal point
        of the Robo's South.

        :return: int -> representing a cardinal point from constants.py
        """
        if self.orientation == consts.NORTH or self.orientation == consts.EAST:
            cardinal_point = self.orientation + 2
        else:
            cardinal_point = self.orientation - 2

        return cardinal_point

    def __update_pos(self, heading):
        """
        Updates the Robo's positional values and orientation based on the provided
        cardinal point heading.

        :param heading: int -> representing a cardinal point from constants.py
        :return: None
        """
        if heading == consts.EAST:
            self.pos_i += 1
        elif heading == consts.SOUTH:
            self.pos_j += 1
        elif heading == consts.WEST:
            self.pos_i -= 1
        elif heading == consts.NORTH:
            self.pos_j -= 1
        else:
            raise ValueError("""
                Unknown heading provided. Please use one from constants.py.\n
                __update_pos(self, heading) : heading == {}
            """.format(heading))

        self.orientation = heading

    def __update_adjacent_map_nodes(self, map_node):
        """
        This method is so that print still works. Otherwise, only East and South walls
        of visited map nodes get printed.

        :param map_node: MapNode -> the node triggering the update
        :return: None
        """
        if map_node.walls[consts.WEST] and map_node.i > 0:
            # update West node's East wall
            west_node = self.maze_map.get_map_node_at_pos(map_node.j, map_node.i - 1)
            west_node.walls[consts.EAST] = True  # TODO: Fugly.

        if map_node.walls[consts.NORTH] and map_node.j > 0:
            # update North node's South wall
            north_node = self.maze_map.get_map_node_at_pos(map_node.j - 1, map_node.i)
            north_node.walls[consts.SOUTH] = True  # TODO: Fugly.

    def __raise_arm_step(self, arm):
        if arm == consts.LEFT_ARM:
            self.boundary.raise_arm_left_step(consts.ARM_STEP_SIZE_DEG)
        elif arm == consts.RIGHT_ARM:
            self.boundary.raise_arm_right_step(consts.ARM_STEP_SIZE_DEG)
        else:
            raise ValueError("""
                Fuck you.
            """)

    def __lower_arm_step(self, arm):
        if arm == consts.LEFT_ARM:
            self.boundary.lower_arm_left_step(consts.ARM_STEP_SIZE_DEG)
        elif arm == consts.RIGHT_ARM:
            self.boundary.lower_arm_right_step(consts.ARM_STEP_SIZE_DEG)
        else:
            raise ValueError("""
                Fuck you.
            """)

    def __get_vel_x(self):
        return self.vel_x

    def __get_vel_y(self):
        return self.vel_y

    def __turn_right_on_spot(self, direction):
        self.boundary.turn_right_on_spot(consts.NOMINAL_VELOCITY, consts.ANGULAR_POINTS[direction])

    def __accelerate_forward(self, turn):
        if math.fabs(self.vel_y - consts.ACCELERATION) > consts.VELOCITY_THRESHOLD:
            self.vel_y = -consts.VELOCITY_THRESHOLD
        else:
            self.vel_y -= consts.ACCELERATION

        if turn == "left":
            self.vel_x = -consts.VELOCITY_THRESHOLD
            right_wheel_comp = -vec.get_hypotenuse_component(self.vel_x, self.vel_y)
            left_wheel_comp = self.vel_y

        elif turn == "right":
            self.vel_x = -consts.VELOCITY_THRESHOLD
            right_wheel_comp = self.vel_y
            left_wheel_comp = -vec.get_hypotenuse_component(self.vel_x, self.vel_y)

        else:
            self.vel_x = 0
            right_wheel_comp = self.vel_y
            left_wheel_comp = self.vel_y

        self.boundary.set_left_motor_velocity(left_wheel_comp)
        self.boundary.set_right_motor_velocity(right_wheel_comp)

    def __accelerate_backward(self, turn):
        if self.vel_y + consts.ACCELERATION > consts.VELOCITY_THRESHOLD:
            self.vel_y = consts.VELOCITY_THRESHOLD
        else:
            self.vel_y += consts.ACCELERATION

        if turn == "left":
            self.vel_x = consts.VELOCITY_THRESHOLD
            right_wheel_comp = vec.get_hypotenuse_component(self.vel_x, self.vel_y)
            left_wheel_comp = self.vel_y

        elif turn == "right":
            self.vel_x = consts.VELOCITY_THRESHOLD
            right_wheel_comp = self.vel_y
            left_wheel_comp = vec.get_hypotenuse_component(self.vel_x, self.vel_y)

        else:
            self.vel_x = 0
            right_wheel_comp = self.vel_y
            left_wheel_comp = self.vel_y

        self.boundary.set_left_motor_velocity(left_wheel_comp)
        self.boundary.set_right_motor_velocity(right_wheel_comp)

    def __decelerate(self):
        """
        Function to __decelerate the robo in whichever direction it is currently traveling.
        :return: None
        """
        # TODO : Using self.vel_y as speed for now.
        if self.vel_y != 0:
            if self.vel_y < math.fabs(consts.ACCELERATION):
                self.vel_y = 0
            else:
                self.vel_y = self.vel_y + consts.ACCELERATION if self.vel_y < 0 else self.vel_y - consts.ACCELERATION
        self.boundary.set_left_motor_velocity(self.vel_y)
        self.boundary.set_right_motor_velocity(self.vel_y)
