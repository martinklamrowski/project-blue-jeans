from MazeGen import generateMaze
from Robo import Robo
from PathFinding import PathFinding

HEIGHT = 10
WIDTH = 10

startY, startX = generateMaze()

robo = Robo(startY,startX)
path = PathFinding()

# Exploring:
proxyDists = robo.pullSensors()