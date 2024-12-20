from commands2 import StartEndCommand, Subsystem
from wpilib import AddressableLED, RobotBase

from enum import Enum

from constants import Constants
from subsystems.leds.patterns import *
from subsystems.leds.zone_buffer import ZonedAddressableLEDBuffer
    
class PatternLevel(Enum):
    DEFAULT = 0
    INTAKE_STATE = 1
    SCORE_STATE = 2

class Zone(Enum):
    MAIN = 0

class LedTriager:

    def __init__(self) -> None:
        self.patterns: list[LedPattern] = [None for _ in range(len(PatternLevel))]
        self.default_pattern = LedPatternRainbow(1)
        self.is_new = True

    def get_pattern(self) -> LedPattern:
        patterns = self.patterns
        for i in range(len(PatternLevel)):
            try:
                if patterns[len(patterns) - 1 - i] is not None:
                    return patterns[len(patterns) - 1 - i]
            except IndexError:
                pass
        return self.default_pattern
    
    def should_refresh(self) -> bool:
        val = self.is_new or self.get_pattern().is_dynamic
        self.is_new = False
        return val
    
    def clear_pattern(self, criticality: PatternLevel) -> None:
        self.patterns[criticality.value] = None
        self.is_new = True

    def add_pattern(self, pattern: LedPattern, criticality: PatternLevel) -> bool:
        self.patterns[criticality.value] = pattern
        self.is_new = True
        return self.get_pattern() == pattern


class LedSubsystem(Subsystem):

    patterns = [LedTriager() for _ in range(len(Zone))]

    def __init__(self):
        self.led_buffer = [AddressableLED.LEDData() for _ in range(Constants.LedConstants.k_led_length)]
        self.strip = AddressableLED(Constants.LedConstants.k_led_pwm_port)
        self.buffers = [
        ZonedAddressableLEDBuffer(self.led_buffer, 0, Constants.LedConstants.k_led_length, False)
    ]

        self.strip.setLength(len(self.led_buffer))
        self.strip.setData(self.led_buffer)
        self.strip.start()
    
    def periodic(self):
        try:
            for i in range(len(Zone)):
                if self.patterns[i].should_refresh():
                    self.patterns[i].get_pattern().apply(self.buffers[i])
            self.strip.setData(self.led_buffer)
        except:
            if not RobotBase.isReal():
                raise

    def set_pattern(self, zone: Zone, pattern: LedPattern, priority: PatternLevel) -> None:
        self.patterns[zone.value].add_pattern(pattern, priority)
        pattern.set_length(self.buffers[zone.value].get_length())

    def clear_pattern(self, zone: Zone, priority: PatternLevel) -> None:
        self.patterns[zone.value].clear_pattern(priority)

    def show_pattern_command(self, pattern: LedPattern, priority: PatternLevel) -> StartEndCommand:
        return StartEndCommand(
            lambda: self.set_pattern(Zone.MAIN, pattern, priority),
            lambda: self.clear_pattern(Zone.MAIN, priority)
        ).ignoringDisable(True)
