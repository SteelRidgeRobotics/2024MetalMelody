from commands2 import Subsystem
import phoenix6
from phoenix6.controls import MotionMagicDutyCycle, DutyCycleOut
from phoenix6.hardware import TalonFX
from phoenix6.signals import ForwardLimitValue
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
        config.slot0.with_k_p(PivotConstants.K_P).with_k_i(PivotConstants.K_I).with_k_d(PivotConstants.K_D)
        config.feedback.with_rotor_to_sensor_ratio(PivotConstants.GEAR_RATIO)
        config.motion_magic.with_motion_magic_acceleration(PivotConstants.MM_ACCELERATION).with_motion_magic_cruise_velocity(PivotConstants.MM_CRUISE_VEL)
        self.pivotMotor.configurator.apply(config)
        self.pivotMotor.set_position(0)
        
        self.has_note = False

    def disencumber(self) -> None:
        self.intakeMotor.set_control(DutyCycleOut(self.slew.calculate(-IntakeConstants.INTAKESPEED)))

    def consume(self) -> None:
        self.intakeMotor.set_control(DutyCycleOut(self.slew.calculate(IntakeConstants.INTAKESPEED)))

    def hold(self) -> None:
        self.intakeMotor.set_control(DutyCycleOut(0))
        
    def periodic(self) -> None:
        if self.intakeMotor.get_forward_limit().value is ForwardLimitValue.CLOSED_TO_GROUND:
            self.has_note = True
        else:
            self.has_note = False
            
    def hasNote(self) -> bool:
        return self.has_note

    def pivotDown(self) -> None:
        self.pivotMotor.set_control(MotionMagicDutyCycle(PivotConstants.INTAKEPOS))

    def pivotStow(self) -> None:
        self.pivotMotor.set_control(MotionMagicDutyCycle(PivotConstants.STOWPOS))

    def pivotAmp(self) -> None:
        self.pivotMotor.set_control(MotionMagicDutyCycle(PivotConstants.SCOREPOS))
