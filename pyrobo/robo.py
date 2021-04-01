import math
import keyboard

import utils.constants as consts
import utils.vec as vec


class Robo(object):
    def __init__(self, boundary=None, nav=None, manual=False):
        print("Well hello there.")

        self.vel_x = 0
        self.vel_y = 0
        self.orientation = consts.EAST
        self.mode = consts.EXPLORATION_MODE
        self.manual = manual

        # interface with Coppelia
        if boundary is None:
            raise TypeError("""
                Sorry, Robo needs a Boundary object; you haven't supplied one.\n
                __init__(self, boundary, nav, manual) : boundary == None
            """)
        self.boundary = boundary

        # Robo's nav object
        if nav is None:
            raise TypeError("""
                Sorry, Robo needs a Navigation object; you haven't supplied one.\n
                __init__(self, boundary, nav, manual) : nav == None
            """)
        self.nav = nav

    def run(self):
        """
        The control loop for the Robo. Call this function to make him
        go!

        :return: None
        """
        # get those noodles up!
        self.__reset_arms()

        if not self.manual:
            # Exploring:
            found_pants = False
            while True:
                # collect data:
                proxy_data = self.__get_surroundings()
                self.nav.display()
                moves = self.nav.getnextPos(proxy_data)
                # move:
                for m in moves:
                    found_pants = self.__move(m)
                    if found_pants:
                        break
                        
                if found_pants:
                    break

            moves = self.nav.goToExit()
            for m in moves:
                self.__move(m)

            self.__dance()
            self.boundary.close_sim_connection()

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
                elif keyboard.is_pressed("1"):
                    self.__snap_to_cardinal_point(consts.EAST)
                elif keyboard.is_pressed("2"):
                    self.__snap_to_cardinal_point(consts.SOUTH)
                elif keyboard.is_pressed("0"):
                    self.__snap_to_cardinal_point(consts.NORTH)
                elif keyboard.is_pressed("3"):
                    self.__snap_to_cardinal_point(consts.WEST)
                elif keyboard.is_pressed("t"):
                    self.__print_robo_orientation()
                elif keyboard.is_pressed("m"):
                    self.__step_forward(1)
                #  TODO : Vision sensor testing; remove.
                elif keyboard.is_pressed("="):
                    self.boundary.get_vision("ortho")
                else:
                    self.__decelerate()

    def __move(self, move):
        """
        Initiate the next move for the Robo. Depending on the character
        supplied, the Robo will either move forward or turn.

        :param move: str -> character specifying the move to make.
        :return: bool -> should we exit the routine; has the Robo found the
                         pants?
        """
        if move == "L":
            snap_point = self.__get_left_cardinality()
            self.__snap_to_cardinal_point(snap_point)
        elif move == "R":
            snap_point = self.__get_right_cardinality()
            self.__snap_to_cardinal_point(snap_point)
        elif move == "F":
            self.__step_forward(1)
        elif move == "C":
            pass
            #  TODO : What is needed here?
            # if self.boundary.get_vision():
            #     self.__pick_up()
            #     return True
        return False

    def __pick_up(self):
        #  TODO : Ya mon.
        # since vision sensor only detects 1 in front, pickup() will
        # need to move forward to pick up the pants
        pass

    def __dance(self):
        #  TODO : :).
        for AYO_MUTHA_LOVA in range(int(6.9)):
            print("DANCINGGGG")

    def __get_surroundings(self):
        """
        Function to get surroundings based on proxie readings.

        :return: list() -> containing the detection states of each sensor.
        """
        left_reading = self.boundary.get_proxie(proxie_name="left_proxie")
        front_reading = self.boundary.get_proxie(proxie_name="front_proxie")
        right_reading = self.boundary.get_proxie(proxie_name="right_proxie")

        surroundings = (
            0 if left_reading is not None else None,
            0 if front_reading is not None else None,
            0 if right_reading is not None else None
        )
        return surroundings

    def __get_left_cardinality(self):
        """
        Returns the cardinal point to the Robo's left. I.e., the true cardinal
        point of the Robo's West.

        :return: int -> representing a cardinal point from constants.py
        """
        if self.orientation == consts.NORTH:
            cardinal_point = consts.WEST
        else:
            cardinal_point = self.orientation - 1

        return cardinal_point

    def __get_right_cardinality(self):
        """
        Returns the cardinal point to the Robo's right. I.e., the true cardinal
        point of the Robo's East.

        :return: int -> representing a cardinal point from constants.py
        """
        if self.orientation == consts.WEST:
            cardinal_point = consts.NORTH
        else:
            cardinal_point = self.orientation + 1

        return cardinal_point

    def __get_rear_cardinality(self):
        """
        Returns the cardinal point to the Robo's rear. I.e., the true cardinal
        point of the Robo's South.

        :return: int -> representing a cardinal point from constants.py
        """
        if self.orientation == consts.NORTH or self.orientation == consts.EAST:
            cardinal_point = self.orientation + 2
        else:
            cardinal_point = self.orientation - 2

        return cardinal_point

    def __raise_arm_step(self, arm):
        """
        Raise the specified arm by the constant step amount defined in
        constants.py.

        :param arm: int -> constant from constants.py corresponding to an arm.
        :return: None
        """
        if arm == consts.LEFT_ARM:
            self.boundary.raise_arm_left_step(consts.ARM_STEP_SIZE_DEG)
        elif arm == consts.RIGHT_ARM:
            self.boundary.raise_arm_right_step(consts.ARM_STEP_SIZE_DEG)
        else:
            raise ValueError("""
                You gave me some whack stuff yo.
            """)

    def __lower_arm_step(self, arm):
        """
        Lower the specified arm by the constant step amount defined in
        constants.py.

        :param arm: int -> constant from constants.py corresponding to an arm.
        :return: None
        """
        if arm == consts.LEFT_ARM:
            self.boundary.lower_arm_left_step(consts.ARM_STEP_SIZE_DEG)
        elif arm == consts.RIGHT_ARM:
            self.boundary.lower_arm_right_step(consts.ARM_STEP_SIZE_DEG)
        else:
            raise ValueError("""
                You gave me some whack stuff yo.
            """)

    def __lower_arms(self):
        """
        Set arms to their lowered position; defined in constants.py.

        :return: None
        """
        self.boundary.set_arm_left_pos(consts.ARM_POSITION_THRESHOLD[0])
        self.boundary.set_arm_right_pos(consts.ARM_POSITION_THRESHOLD[0])

    def __reset_arms(self):
        """
        Reset arms to their up position; defined in constants.py.

        :return: None
        """
        self.boundary.set_arm_left_pos(consts.ARM_POSITION_THRESHOLD[1])
        self.boundary.set_arm_right_pos(consts.ARM_POSITION_THRESHOLD[1])

    def __get_vel_x(self):
        """
        Get the Robo's x-component velocity.

        :return: float -> the x-component of the velocity.
        """
        return self.vel_x

    def __get_vel_y(self):
        """
        Get the Robo's y-component velocity.

        :return: float -> the y-component of the velocity.
        """
        return self.vel_y

    def __snap_to_cardinal_point(self, cardinal_point):
        """
        Snap the Robo to the specified cardinal point.

        :param cardinal_point: int -> the cardinal point as defined in constants.py.
        :return: None
        """
        self.boundary.snap_to_angular_point(consts.TURN_VELOCITY,
                                            consts.ANGULAR_POINTS[cardinal_point])
        self.orientation = cardinal_point

    def __step_forward(self, num_steps):
        """
        Move forward. For use when robo is autonomous.

        :param num_steps: int -> the number of steps (blocks) to move forward.
        :return: None
        """
        for _ in range(num_steps):
            self.boundary.step_forward(consts.NOMINAL_VELOCITY,
                                       consts.ANGULAR_POINTS[self.orientation])

    def __accelerate_forward(self, turn):
        """
        Move forward. For use when controlling the Robo.

        :param turn: str -> 'left' or 'right'.
        :return: None
        """
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
        """
        Move backwards. For use when controlling the Robo.

        :param turn: str -> 'left' or 'right'.
        :return: None
        """
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
        Function to decelerate the Robo in whichever direction it is currently
        traveling.

        :return: None
        """
        #  TODO : Using self.vel_y as speed for now.
        if self.vel_y != 0:
            if self.vel_y < math.fabs(consts.ACCELERATION):
                self.vel_y = 0
            else:
                self.vel_y = self.vel_y + consts.ACCELERATION if self.vel_y < 0 else self.vel_y - consts.ACCELERATION
        self.boundary.set_left_motor_velocity(self.vel_y)
        self.boundary.set_right_motor_velocity(self.vel_y)

    def __print_robo_orientation(self):
        #  TODO : Testing; remove.
        stuff = self.boundary.get_orientation("body")
        print(stuff)
        print(vec.euler_g_to_rad(stuff[2]))
