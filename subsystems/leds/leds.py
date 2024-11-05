import wpilib
from commands2 import Subsystem

class LED(Subsystem):
    def __init__(self):
        
        self.led = wpilib.AddressableLED(1)

        self.buffer = [144]
        self.led.setLength(144)

        self.led.setData(self.buffer)
        self.led.start()

    def setRGB(self, red, green, blue):
        
        self.led.LEDData.setRGB(red, green, blue)






