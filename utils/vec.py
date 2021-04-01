"""
Some general maths functions.
"""
import math


def get_hypotenuse_component(x, y):
    """
    Pythagoras in da haussssss.

    :param x: float -> AKA a
    :param y: float -> AKA b
    :return: float -> hypotenuse, AKA c
    """

    return math.sqrt((x ** 2 + y ** 2))


def euler_g_to_rad(g):
    """
    Euler angles, search engine it.

    :param g: float -> gamma in gamma
    :return: float -> gamma in rad
    """

    return ((g + math.pi) % (2 * math.pi)) - math.pi if g >= 0 \
        else ((g - math.pi) % (2 * math.pi)) + math.pi


def get_closer_rotation_direction(angular_point1, angular_point2):
    """
    Function that computes the shortest direction to rotate from
    point 1 to point 2.

    :param angular_point1: float -> point to rotate from.
    :param angular_point2: float -> target point.
    :return: bool -> True if should rotate counterclockwise, False
                     if should rotate clockwise.
    """
    return (angular_point1 - angular_point2 + (2 * math.pi)) % (2 * math.pi) > math.pi
