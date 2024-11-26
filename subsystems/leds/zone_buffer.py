from wpilib import AddressableLED, Color, Color8Bit

from typing import overload

class ZonedAddressableLEDBuffer:

    def __init__(self, buffer: list[AddressableLED.LEDData], start: int, end: int, is_inverted: bool) -> None:

        if start > end:
            raise ValueError("Start must be less than end.")
        
        self.buffer = buffer
        self.is_inverted = is_inverted
        self.start = start
        self.length = end - start

    def set_RGB(self, index: int, r: int, g: int, b: int) -> None:
        self.buffer[self._get_absolute_index(index)].setRGB(r, g, b)

    def set_HSV(self, index: int, h: int, s: int, v: int) -> None:
        self.buffer[self._get_absolute_index(index)].setHSV(h, s, v)

    @overload
    def set_LED(self, index: int, color: Color) -> None:
        self.buffer[self._get_absolute_index(index)].setLED(color)

    def set_LED(self, index: int, color: Color8Bit) -> None:
        self.buffer[self._get_absolute_index(index)].setLED(color)

    def get_LED(self, index: int) -> Color:
        led = self.buffer[self._get_absolute_index(index)]
        return Color(led.r, led.g, led.b)
    
    def get_LED_8_bit(self, index: int) -> Color:
        led = self.buffer[self._get_absolute_index(index)]
        return Color8Bit(led.r, led.g, led.b)
    
    def get_length(self) -> int:
        return self.length

    def _get_absolute_index(self, index: int) -> int:
        if index >= self.length:
            return 0
        
        if self.is_inverted:
            return self.start + self.length - 1 - index
        else:
            return self.start + index
