from commands2 import Command, Subsystem
from enum import Enum
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import DutyCycleOut
from phoenix6.hardware import TalonFX
from phoenix6.signals import ForwardLimitValue
from constants import *
import wpilib

class ScoreStates(Enum):
    HOLD = 0
    MOVE_TO_INDEXER = 1

class Intake(Subsystem):

    state = ScoreStates.MOVE_TO_INDEXER

    def __init__(self):
        super().__init__()
        self.setName("Intake")
        
        self.intakeMotor = TalonFX(MotorIDs.INTAKEMOTOR)
        intake_config = TalonFXConfiguration()
        intake_config.motor_output.with_neutral_mode(NeutralModeValue.BRAKE)
        intake_config.feedback.sensor_to_mechanism_ratio = IntakeConstants.GEAR_RATIO
        self.intakeMotor.configurator.apply(intake_config)
        
        self.has_note = False
        self.beam_breaker = wpilib.DigitalInput(1)

    def disencumber(self) -> None:
        self.intakeMotor.set_control(DutyCycleOut(-0.5, enable_foc=False))

    def consume(self) -> None:
        self.intakeMotor.set_control(DutyCycleOut(IntakeConstants.INTAKESPEED))

    def stop(self) -> None:
        self.intakeMotor.set_control(DutyCycleOut(0))
        
    def periodic(self) -> None:

        self.has_note = self.beam_breaker.get()

        if self.has_note and self.state == ScoreStates.HOLD:
            self.stop()

    def toggle_hold_note(self) -> None:
        self.state = not self.state
            
    def hasNote(self) -> bool:
        return self.has_note
