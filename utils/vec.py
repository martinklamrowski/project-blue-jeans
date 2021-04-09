"""
Some general maths functions.
"""
import math


def get_hypotenuse(x, y):
    """Pythagoras in da haussssss.

    :param x: AKA a.
    :type x: float
    :param y: AKA b.
    :type y: float
    :return: Hypotenuse, AKA c.
    :rtype: float
    """

    return math.sqrt((x ** 2 + y ** 2))


def euler_g_to_rad(g):
    """Euler angles, search engine it.

    :param g: Gamma in gamma.
    :type g: float
    :return: Gamma in rad.
    :rtype: float
    """

    return ((g + math.pi) % (2 * math.pi)) - math.pi if g >= 0 \
        else ((g - math.pi) % (2 * math.pi)) + math.pi


def get_closer_rotation_direction(angular_point1, angular_point2):
    """Function that computes the shortest direction to rotate from
    point 1 to point 2.

    :param angular_point1: Point to rotate from.
    :type angular_point1: float
    :param angular_point2: Target point.
    :type angular_point2: float
    :return: True if should rotate counterclockwise, False if should rotate clockwise.
    :rtype: bool
    """
    return (angular_point1 - angular_point2 + (2 * math.pi)) % (2 * math.pi) > math.pi


def is_similar_colour(rgb1, rgb2):
    """
    Custom colour similarity metric. Fancy huh?

    :param rgb1: A colour.
    :type rgb1: list(int, int, int)
    :param rgb2: Another colour.
    :type rgb2: list(int, int, int)
    :return:
    """

    if math.fabs(rgb1[0] - rgb2[0]) < 55 and math.fabs(rgb1[1] - rgb2[1]) < 55 and math.fabs(rgb1[2] - rgb2[2]) < 55:
        return True
    return False
