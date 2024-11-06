from commands2 import Command, StartEndCommand, Subsystem
from wpilib import AddressableLED, Color, Color8Bit, DataLogManager, RobotBase
from abc import ABC, abstractmethod
import colorsys
from enum import Enum
import traceback

from constants import Constants

class LedSubsystem(Subsystem):

    class Zone(Enum):
        MAIN = 0
    k_zone_count = len(Zone)

    class PatternLevel(Enum):
        DEFAULT = 0
        INTAKE_STATE = 1
        SCORE_STATE = 2
    k_level_count = len(PatternLevel)
        
    def __init__(self):
        self.strip = AddressableLED(Constants.LEDConstants.k_led_pwm_port)

        self.led_buffer = [AddressableLED.LEDData() for i in range(Constants.LEDConstants.k_led_length)]

        self.buffers = [
            ZonedAddressableLEDBuffer(self.led_buffer, 0, 143, False)
        ]

        self.patterns = [LedAllocation() for i in range(LedSubsystem.k_zone_count)]

        self.strip.setLength(Constants.LEDConstants.k_led_length)
        self.strip.setData(self.led_buffer)
        self.strip.start()

    def periodic(self):
        try:
            for i in range(LedSubsystem.k_zone_count):
                if self.patterns[i].should_refresh():
                    self.patterns[i].get_pattern().apply(self.buffers[i])
            self.strip.setData(self.led_buffer)
        except Exception as e:
            if RobotBase.isReal():
                DataLogManager.log(f"Error in LEDs periodic: {traceback.format_exc()}")
            else:
                raise e
            
    def set_pattern(self, zone: Zone, pattern: "LedPatternBase", priority: PatternLevel) -> None:
        self.patterns[zone.value].add_pattern(pattern, priority)
        pattern.set_length(self.buffers[zone.value].get_length())

    def clear_pattern(self, zone: Zone, priority: PatternLevel) -> None:
        self.patterns[zone.value].clear_pattern(priority)

    def show_pattern_command(self, pattern: "LedPatternBase", priority: PatternLevel) -> Command:
        return StartEndCommand(
            lambda: self.set_pattern(LedSubsystem.Zone.MAIN, pattern, priority),
            lambda: self.clear_pattern(LedSubsystem.Zone.MAIN, priority)
        ).ignoringDisable(True)

class ZonedAddressableLEDBuffer:

    def __init__(self, buffer: list[AddressableLED.LEDData], start: int, end: int, is_inverted: bool) -> None:
        """Creates a buffer to seperate parts of the LED buffer.

        Shoutout 1540 Flaming Chickens :fire:

        Args:
            buffer (list[AddressableLED.LEDData]): The AddressableLED buffer.
            start (int): The start buffer index.
            end (int): The end buffer index.
            is_inverted (bool): Inverts index calls to easily mirror multiple zones.
        """

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
        
class LedPatternBase(ABC):

    def __init__(self, is_dynamic: bool):
        self.is_dynamic = is_dynamic

    @abstractmethod
    def apply(self, buffer: ZonedAddressableLEDBuffer) -> None:
        pass

    def set_length(length: int) -> None:
        pass

    def getHSV(color: Color) -> list[int, int, int]:
        val = colorsys.rgb_to_hsv(color.red, color.green, color.blue)
        return [i * 255 for i in val]
    
class LedPatternRainbow(LedPatternBase):
    initial_hue = 0

    def __init__(self, speed: int) -> None:
        super().__init__(True)
        self.speed = speed

    def apply(self, buffer: ZonedAddressableLEDBuffer):
        for i in range(buffer.get_length()):
            hue = (LedPatternRainbow.initial_hue + (i * 180 / buffer.get_length())) % 180
            buffer.set_HSV(i, int(hue),255,255)

            LedPatternRainbow.initial_hue += 1
            LedPatternRainbow.initial_hue %= 180
    
class LedAllocation:

    patterns: list["LedPatternBase"] = []

    k_default_pattern = LedPatternRainbow(0.001)
    is_new = True

    def get_pattern(self) -> "LedPatternBase":
        patterns = LedAllocation.patterns
        for i in range(LedSubsystem.k_level_count):
            try:
                if patterns[len(patterns) - 1 - i] is not None:
                    return patterns[len(patterns) - 1 - i]
            except IndexError:
                pass
        return LedAllocation.k_default_pattern
    
    def should_refresh(self) -> bool:
        val = self.is_new or self.get_pattern().is_dynamic
        self.is_new = False
        return val
    
    def clear_pattern(criticality: LedSubsystem.PatternLevel) -> None:
        LedAllocation.patterns[criticality.value] = None
        LedAllocation.is_new = True

    def add_pattern(pattern: "LedPatternBase", criticality: LedSubsystem.PatternLevel) -> bool:
        LedAllocation.patterns[criticality.value] = pattern
        LedAllocation.is_new = True
        return LedAllocation.get_pattern() == pattern


