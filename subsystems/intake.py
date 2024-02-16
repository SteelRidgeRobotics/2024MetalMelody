from commands2 import Subsystem
from constants import *
import phoenix6
from phoenix6.controls import MotionMagicDutyCycle, DutyCycleOut
from phoenix6.hardware import TalonFX
from wpimath.filter import SlewRateLimiter

class Intake(Subsystem):
    slew = SlewRateLimiter(.5)
    
    def __init__(self):
        super().__init__()
        
        self.intakeMotor = TalonFX(MotorIDs.INTAKEMOTOR)
        self.pivotMotor = TalonFX(MotorIDs.PIVOTMOTOR)
        config = phoenix6.configs.TalonFXConfiguration()
        config.motor_output.with_neutral_mode(phoenix6.configs.config_groups.NeutralModeValue.COAST)
        config.slot0.with_k_p(1)
        config.feedback.sensor_to_mechanism_ratio = 12
        config.motion_magic.with_motion_magic_acceleration(IntakeConstants.MM_ACCELERATION).with_motion_magic_cruise_velocity(IntakeConstants.MM_CRUISE_VEL)
        self.pivotMotor.configurator.apply(config)

        self.pivotMotor.set_position(0)

    def disencumber(self) -> None: # drop note
        self.intakeMotor.set_control(DutyCycleOut(self.slew.calculate(-IntakeConstants.INTAKESPEED * 1.5)))

    def consume(self) -> None: # intake note
        self.intakeMotor.set_control(DutyCycleOut(self.slew.calculate(IntakeConstants.INTAKESPEED)))

    def hold(self) -> None: # hold note
        self.intakeMotor.set_control(DutyCycleOut(0))

    def pivotDown(self) -> None:
        self.pivotMotor.set_control(MotionMagicDutyCycle(IntakeConstants.INTAKEPOS))

    def pivotStow(self) -> None:
        self.pivotMotor.set_control(MotionMagicDutyCycle(IntakeConstants.STOWPOS))

    def pivotAmp(self) -> None:
        self.pivotMotor.set_control(MotionMagicDutyCycle(IntakeConstants.SCOREPOS))
