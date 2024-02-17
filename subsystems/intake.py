from commands2 import Subsystem
import phoenix6
from phoenix6.controls import MotionMagicDutyCycle, DutyCycleOut
from phoenix6.hardware import TalonFX
from wpimath.filter import SlewRateLimiter
from constants import *

class Intake(Subsystem):
    slew = SlewRateLimiter(.5)
    
    def __init__(self):
        super().__init__()
        
        self.intakeMotor = TalonFX(MotorIDs.INTAKEMOTOR)
        self.pivotMotor = TalonFX(MotorIDs.PIVOTMOTOR)
        config = phoenix6.configs.TalonFXConfiguration()
        config.motor_output.with_neutral_mode(phoenix6.configs.config_groups.NeutralModeValue.BRAKE)
        config.slot0.with_k_p(2.6).with_k_i(0.64).with_k_d(0.1).with_gravity_type(phoenix6.configs.config_groups.GravityTypeValue.ARM_COSINE)
        config.feedback.sensor_to_mechanism_ratio = 12
        config.motion_magic.with_motion_magic_acceleration(IntakeConstants.MM_ACCELERATION).with_motion_magic_cruise_velocity(IntakeConstants.MM_CRUISE_VEL)
        self.pivotMotor.configurator.apply(config)

        self.pivotMotor.set_position(0)

    def disencumber(self) -> None: # drop note
        self.intakeMotor.set_control(DutyCycleOut(self.slew.calculate(-IntakeConstants.INTAKESPEED)))

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
