from robo_STUB import RoboSTUB
import time

class Robo:
    def __init__(self, startY, startX, STUB=False, STUBmazeY=0, STUBmazeX=0):
        if STUB:    self.roboSTUB = RoboSTUB(startY, startX, STUBmazeY, STUBmazeX)
        self.STUB = STUB
        print('heyo from Robo')

        # TODO: open coppelia shitttt yo


    def pullProxy(self):
        if self.STUB:   return self.roboSTUB.proxySensors()

    def move(self, move):
        if move == 'L':
            if self.STUB: self.roboSTUB.left()
        elif move == 'R':
            if self.STUB: self.roboSTUB.right()
        elif move == 'F':
            if self.STUB: self.roboSTUB.forward()
        elif move == 'C':
            if self.__pullVision():
                self.pickUp()
                return True
        return False

    def pickUp(self):
        if self.STUB: self.roboSTUB.forward() # since vision sensor only detects 1 in front, pickup() will need to move forward to pick up the pants
        print('woohoo!')

    def dance(self):
        for AYO_MUTHA_FUCKA in range(int(6.9)):
            print('DANCINGGGG')

    def __pullVision(self):
        if self.STUB: return self.roboSTUB.pullVision()
        return False