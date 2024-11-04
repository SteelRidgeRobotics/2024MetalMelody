from commands2 import Subsystem
from enum import Enum
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import DutyCycleOut
from phoenix6.hardware import TalonFX
from phoenix6.signals import ForwardLimitValue
import phoenix6
import wpilib

from constants import *

class Indexer(Subsystem):
    
    def __init__(self):
        super().__init__()
        
        self.motor = TalonFX(MotorIDs.INDEXER_MOTOR)

        self.beam_breaker = wpilib.DigitalInput(2)

        self.has_note = False
        self.is_shooting = False



    def swallow(self) -> None:
        self.motor.set_control(DutyCycleOut(LauncherConstants.INDEXERSPEED))
        self.is_shooting = True
        wpilib.SmartDashboard.putBoolean("Swallowing", True)

    def stop(self) -> None:
        self.motor.set_control(DutyCycleOut(0))
        self.is_shooting = False
        wpilib.SmartDashboard.putBoolean("Swallowing", False)

    def periodic(self) -> None:
        self.has_note = self.beam_breaker.get()

        if self.has_note and not self.is_shooting:
            self.stop()

        return super().periodic()
