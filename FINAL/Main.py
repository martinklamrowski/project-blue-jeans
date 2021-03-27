from MazeGen import generateMaze
from Robo import Robo
from PathFinding import PathFinding

import os
import time
import numpy as np

COPPELIA_STUB = True

HEIGHT = 5
WIDTH = 5

if not COPPELIA_STUB:
    startY, startX = generateMaze(HEIGHT, WIDTH)
else:
    startY, startX = 1, 0

robo = Robo(startY,startX, COPPELIA_STUB, HEIGHT, WIDTH)
path = PathFinding(HEIGHT, WIDTH, startY, startX)



# Exploring:
while True:
    # collect data:
    proxyData = robo.pullSensors()
    if len(proxyData) == 4:
        break
    moves = path.getnextPos(proxyData)

    if COPPELIA_STUB:
        print('ROBO:\t\tPATH:')
        mapRobo = robo.roboSTUB.map.copy()
        mapPath = path.map.copy()
        mapRobo[robo.roboSTUB.currY, robo.roboSTUB.currX] = 4
        mapPath[path.currY, path.currX] = 4
        print('\t', moves)
        for h in range(HEIGHT):
            print(mapRobo[h,:],'\t', mapPath[h, :])
        time.sleep(1)
    # move:
    while len(moves) > 0:
        robo.move(moves.pop())
        time.sleep(1)
    print()


# TODO: add a 'just check vision sensor option' for when going over known 2 tiles



robo.pickUp()

moves = path.goToExit() # should this take proxy data too?
while len(moves) > 0:
    os.system('cls')
    print(moves.pop())
    time.sleep(1)

robo.dance()
time.sleep(3)
robo.closeConnection()