import sim_lib.sim as sim
import sim_lib.simConst as sC
import utils.maze as m

print('Program started')
sim.simxFinish(-1)  # just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1', 8008, True, True, 5000, 5)  # Connect to CoppeliaSim

if clientID != -1:
    print('Connected to remote API server')

    LENGTH = 30  # Number of blocks to use for length
    WIDTH = 30  # Number of blocks to use for width
    OPENING = 'west'
    new_maze = m.Maze(LENGTH, WIDTH, OPENING)
    new_maze.generate_maze()  # Generates random maze
    new_maze.generate_object()  # Generates object position at random deadend

    emptyBuff = bytearray()
    res, retInts, retFloats, retStrings, retBuffer = sim.simxCallScriptFunction(clientID,
                                                                                '',
                                                                                sC.sim_scripttype_childscript,
                                                                                'render@ResizableFloor_5_25',
                                                                                [LENGTH, WIDTH],
                                                                                [],
                                                                                new_maze.reduce(),
                                                                                emptyBuff,
                                                                                sC.simx_opmode_blocking)

    if res == sC.simx_return_ok:
        new_maze.render()
        # print ('Dummy handle: ',retInts[0]) # display the reply from CoppeliaSim (in this case, the handle of the created dummy)
    else:
        print('Remote function call failed')

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
print('Program ended')
