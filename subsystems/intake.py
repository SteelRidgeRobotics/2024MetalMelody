from commands2 import Subsystem
from enum import Enum
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import DutyCycleOut
from phoenix6.hardware import TalonFX
from phoenix6.signals import ForwardLimitValue
from commands.mode_toggle import ModeToggle as mode_toggle
from constants import *
import wpilib

class IntakeStates(Enum):
    STOPPED = 0
    DISENCUMBERING = 1
    CONSUME = 2

class Intake(Subsystem):
    
    def __init__(self):
        super().__init__()
        self.setName("Intake")
        
        self.intakeMotor = TalonFX(MotorIDs.INTAKEMOTOR)
        intake_config = TalonFXConfiguration()
        intake_config.motor_output.with_neutral_mode(NeutralModeValue.BRAKE)
        intake_config.feedback.sensor_to_mechanism_ratio = IntakeConstants.GEAR_RATIO
        self.intakeMotor.configurator.apply(intake_config)
        
        self.has_note = False
        self.state = IntakeStates.STOPPED
        self.beam_breaker = wpilib.DigitalInput(1)

    def disencumber(self) -> None:
        if mode_toggle.mode == Modes.INTAKE:
            self.intakeMotor.set_control(DutyCycleOut(-0.5, enable_foc=False))
            self.state = IntakeStates.DISENCUMBERING

    def consume(self) -> None:
        self.intakeMotor.set_control(DutyCycleOut(IntakeConstants.INTAKESPEED))
        self.state = IntakeStates.CONSUME

    def stop(self) -> None:
        self.intakeMotor.set_control(DutyCycleOut(0))
        self.state = IntakeStates.STOPPED
        
    def periodic(self) -> None:

        self.has_note = self.beam_breaker.get()

        if self.has_note and mode_toggle.get_mode() == Modes.INTAKE:
            self.stop()
            
    def hasNote(self) -> bool:
        return self.has_note
