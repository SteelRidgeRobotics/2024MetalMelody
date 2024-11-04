from commands2 import Command, Subsystem
from enum import Enum
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import DutyCycleOut
from phoenix6.hardware import TalonFX
from phoenix6.signals import ForwardLimitValue
from constants import *
import wpilib



class Intake(Subsystem):

    def __init__(self):
        super().__init__()
        self.setName("Intake")
        
        self.intakeMotor = TalonFX(MotorIDs.INTAKEMOTOR)

        intake_config = TalonFXConfiguration()
        intake_config.motor_output.with_neutral_mode(NeutralModeValue.BRAKE)
        intake_config.feedback.sensor_to_mechanism_ratio = IntakeConstants.GEAR_RATIO
        self.intakeMotor.configurator.apply(intake_config)
        
        self.beam_breaker = wpilib.DigitalInput(1)

        wpilib.SmartDashboard.putBoolean("IntakeOn", False)
        wpilib.SmartDashboard.putBoolean("Dropping", False)


    def disencumber(self) -> None:
        self.intakeMotor.set_control(DutyCycleOut(-0.5, enable_foc=False))
        wpilib.SmartDashboard.putBoolean("IntakeOn", True)
        wpilib.SmartDashboard.putBoolean("Dropping", True)

    def consume(self) -> None:
        self.intakeMotor.set_control(DutyCycleOut(IntakeConstants.INTAKESPEED))
        wpilib.SmartDashboard.putBoolean("IntakeOn", True)
        wpilib.SmartDashboard.putBoolean("Dropping", False)

    def stop(self) -> None:
        self.intakeMotor.set_control(DutyCycleOut(0))
        wpilib.SmartDashboard.putBoolean("IntakeOn", False)
        wpilib.SmartDashboard.putBoolean("Dropping", False)
            