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
        
        self.top_motor = TalonFX(MotorIDs.TOP_INDEXER_MOTOR)
        self.bottom_motor = TalonFX(MotorIDs.BOTTOM_INDEXER_MOTOR)

        self.invertConfig = phoenix6.configs.MotorOutputConfigs()
        self.invertConfig.inverted = phoenix6.configs.talon_fx_configs.InvertedValue.CLOCKWISE_POSITIVE
        self.bottom_motor.configurator.apply(self.invertConfig)

        self.beam_breaker = wpilib.DigitalInput()
        
        self.has_note = False
        self.is_shooting = False
        self.state = IndexerStates.STOPPED

    def intakeWise(self) -> None:
        self.top_motor.set_control(DutyCycleOut(-IndexerConstants.INDEXERSPEED, enable_foc=False))
        self.bottom_motor.set_control(DutyCycleOut(-IndexerConstants.INDEXERSPEED, enable_foc=False))
        self.state = IndexerStates.INTAKEWISE

    def launcherWise(self) -> None:
        self.top_motor.set_control(DutyCycleOut(IndexerConstants.INDEXERSPEED))
        self.bottom_motor.set_control(DutyCycleOut(IndexerConstants.INDEXERSPEED, enable_foc=False))
        self.state = IndexerStates.LAUNCHERWISE

    def stop(self) -> None:
        self.top_motor.set_control(DutyCycleOut(0))
        self.bottom_motor.set_control(DutyCycleOut(0))
        self.state = IndexerStates.STOPPED

    def periodic(self) -> None:
        self.has_note = self.beam_breaker.get()

        if self.has_note and not self.is_shooting:
            self.stop()

        return super().periodic()
