from commands2 import Subsystem
from enum import Enum
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import DutyCycleOut
from phoenix6.hardware import TalonFX
from phoenix6.signals import ForwardLimitValue
from constants import *

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

    def disencumber(self) -> None:
        self.intakeMotor.set_control(DutyCycleOut(-0.5, enable_foc=False))
        self.state = IntakeStates.DISENCUMBERING

    def consume(self) -> None:
        self.intakeMotor.set_control(DutyCycleOut(IntakeConstants.INTAKESPEED))
        self.state = IntakeStates.CONSUME

    def stop(self) -> None:
        self.intakeMotor.set_control(DutyCycleOut(0))
        self.state = IntakeStates.STOPPED
        
    def periodic(self) -> None:
        if self.intakeMotor.get_forward_limit().value is ForwardLimitValue.CLOSED_TO_GROUND:
            self.has_note = True
        else:
            self.has_note = False
            
    def hasNote(self) -> bool:
        return self.has_note
