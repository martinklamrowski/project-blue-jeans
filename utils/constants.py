"""
Constant definitions.
"""

import math

# orientation constants
NORTH = 0
EAST = 1
SOUTH = 2
WEST = 3

ANGULAR_POINTS = {
    NORTH: 0,
    EAST: 3 * math.pi/2,
    SOUTH: math.pi,
    WEST: math.pi/2
}

# modes
EXPLORATION_MODE = 11
HOOK_MODE = 12
EXIT_MODE = 13

# some movement parameters
ARM_STEP_SIZE_DEG = 5
ARM_POSITION_THRESHOLD = (0, - 55 * math.pi / 180)
TURN_VELOCITY = 1
NOMINAL_STEP_VELOCITY = 7
NOMINAL_VELOCITY = 0.5
ACCELERATION = 0.01
VELOCITY_THRESHOLD = 15

# CoppeliaSim magic numbers
BLOCK_SIZE = 0.5

# other
LEFT_ARM = 4
RIGHT_ARM = 5
