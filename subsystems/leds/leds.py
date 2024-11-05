import wpilib
from commands2 import Subsystem

class LED(Subsystem):
    def __init__(self):
        length = 144
        self.rainbowHue = 0

        self.led = wpilib.AddressableLED(9)

        self.buffer = []
        for i in range(length):
            self.buffer.append(self.led.LEDData(255, 255, 255))

        print(self.buffer)
        self.led.setLength(length)

        self.led.setData(self.buffer)
        self.led.start()


   # def periodic(self) -> None:
   #         hue = (self.rainbowHue + (i * 180 / self.buffer.getLength())) % 180

    def setRGB(self, red, green, blue):
        for i in range(len(self.buffer)):
            self.buffer[i].setRGB(red, green, blue)

        self.led.setData(self.buffer)
