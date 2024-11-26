from wpilib import Color

from abc import ABC, abstractmethod
import colorsys
from typing import Callable, overload
from random import randint

from subsystems.leds.zone_buffer import ZonedAddressableLEDBuffer

class LedPattern(ABC):

    def __init__(self, is_dynamic: bool):
        self.is_dynamic = is_dynamic

    @abstractmethod
    def apply(self, buffer: ZonedAddressableLEDBuffer) -> None:
        pass

    def set_length(self, length: int) -> None:
        pass

    @staticmethod
    def get_hsv(color: Color) -> list[int, int, int]:
        h, s, v = colorsys.rgb_to_hsv(color.red, color.green, color.blue)
        return [h*180, s*255, v*255]
    
class SimpleLedPattern(LedPattern):

    def __init__(self, applier):
        super().__init__(False)
        self.applier = applier

    def apply(self, buffer: ZonedAddressableLEDBuffer) -> None:
        for i in range(buffer.get_length()):
            self.applier(buffer, i)

    @staticmethod
    @overload
    def solid(color: str) -> LedPattern:
        return SimpleLedPattern.solid(Color(color))
    
    @staticmethod
    def solid(color: Color) -> LedPattern:
        return SimpleLedPattern(lambda buffer, i: buffer.set_LED(i, color))
    
    @staticmethod
    def blank() -> LedPattern:
        return SimpleLedPattern(lambda buffer, i: buffer.set_LED(i, Color.kBlack))
    
class LedPatternRainbow(LedPattern):

    def __init__(self, speed: int) -> None:
        super().__init__(True)
        self.speed = speed
        self.initial_hue = 0

    def apply(self, buffer: ZonedAddressableLEDBuffer):
        for i in range(buffer.get_length()):
            hue = (self.initial_hue + (i * 180 / buffer.get_length())) % 180
            buffer.set_HSV(i, int(hue),255,255)

            self.initial_hue += self.speed / buffer.get_length()
            self.initial_hue %= 180

class LedPatternSeisurizer(LedPattern):

    def __init__(self) -> None:
        super().__init__(True)

    def apply(self, buffer: ZonedAddressableLEDBuffer):
        for i in range(buffer.get_length()):
            buffer.set_HSV(i, randint(0, 180), 255, 255)


class LedPatternPulse(LedPattern):

    def __init__(self, hue: int, speed: int) -> None:
        super().__init__(True)
        self.hue = hue
        self.speed = speed

        self.initial_value = 0

    def apply(self, buffer: ZonedAddressableLEDBuffer):
        for i in range(buffer.get_length()):

            value = (self.initial_value + (i * 255 / buffer.get_length())) % 255
            buffer.set_HSV(i, self.hue, int(value), int(value))
            self.initial_value += self.speed / buffer.get_length()
            self.initial_value %= 255
