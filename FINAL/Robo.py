from robo_STUB import RoboSTUB

class Robo:
    def __init__(self, startY, startX, STUB=False, STUBmazeY=0, STUBmazeX=0):
        print('heyo from Robo')

        # TODO: open coppelia

        if STUB:    self.roboSTUB = RoboSTUB(startY, startX, STUBmazeY, STUBmazeX)
        self.STUB = STUB

    '''
    NOTE: main will know to break loop cause pullSensors() will return proxyData[] with len=4 instead of 3 (like normal)
    '''
    def pullSensors(self):
        visionSensorData = self.__visionSensor()
        # read proxy
        if visionSensorData:
            return [1,1,1,True]
        return self.__proxySensors()

    def move(self, move):
        if self.STUB: self.roboSTUB.move(move)



    def pickUp(self):
        print('woohoo!')

    def __proxySensors(self):
        if self.STUB:   return self.roboSTUB.proxySensors()

    def __visionSensor(self):
        return False