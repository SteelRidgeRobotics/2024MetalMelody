from commands2 import Subsystem
from enum import Enum
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import DutyCycleOut
from phoenix6.hardware import TalonFX
from phoenix6.signals import ForwardLimitValue
import phoenix6
import wpilib

from constants import *

class IndexerStates(Enum):
    STOPPED = 0
    LAUNCHERWISE = 1
    INTAKEWISE = 2

class Indexer(Subsystem):
    
    def __init__(self):
        super().__init__()
        
        self.motor = TalonFX(MotorIDs.INDEXER_MOTOR)

        self.intake_beam_breaker = wpilib.DigitalInput(1)
        self.launcher_beam_breaker = wpilib.DigitalInput(2)

        self.has_note = False
        self.is_shooting = False
        self.state = IndexerStates.STOPPED

    def intakeWise(self) -> None:
        self.motor.set_control(DutyCycleOut(-IndexerConstants.INDEXERSPEED, enable_foc=False))
        self.state = IndexerStates.INTAKEWISE

    def launcherWise(self) -> None:
        self.motor.set_control(DutyCycleOut(IndexerConstants.INDEXERSPEED))
        self.state = IndexerStates.LAUNCHERWISE
        self.is_shooting = True

    def stop(self) -> None:
        self.motor.set_control(DutyCycleOut(0))
        self.state = IndexerStates.STOPPED
        self.is_shooting = False

    def periodic(self) -> None:
        #self.has_note = self.beam_breaker.get()

        if self.has_note and not self.is_shooting:
            self.stop()

        return super().periodic()
