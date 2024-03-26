from commands2 import Subsystem
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import NeutralModeValue
from phoenix6.controls import MotionMagicDutyCycle
from phoenix6.hardware import TalonFX
from constants import *

class Pivot(Subsystem):
    
    def __init__(self) -> None:
        super().__init__()
        self.setName("Pivot")
        
        self.pivotMotor = TalonFX(MotorIDs.PIVOTMOTOR)
        pivot_config = TalonFXConfiguration()
        pivot_config.motor_output.with_neutral_mode(NeutralModeValue.BRAKE).with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
        pivot_config.slot0.with_k_p(PivotConstants.K_P).with_k_i(PivotConstants.K_I).with_k_d(PivotConstants.K_D).with_k_v(PivotConstants.K_V).with_k_s(PivotConstants.K_S)
        pivot_config.feedback.with_sensor_to_mechanism_ratio(PivotConstants.GEAR_RATIO)
        pivot_config.motion_magic.with_motion_magic_acceleration(PivotConstants.MM_ACCELERATION).with_motion_magic_cruise_velocity(PivotConstants.MM_CRUISE_VEL)
        self.pivotMotor.configurator.apply(pivot_config)
        
        self.pivotMotor.set_position(0)
        
    def intake(self) -> None:
        self.pivotMotor.set_control(MotionMagicDutyCycle(PivotConstants.INTAKEPOS))

    def stow(self) -> None:
        self.pivotMotor.set_control(MotionMagicDutyCycle(PivotConstants.STOWPOS))

    def scoreUpwards(self) -> None:
        self.pivotMotor.set_control(MotionMagicDutyCycle(PivotConstants.SCOREPOSUP))

    def scoreDownwards(self) -> None:
        self.pivotMotor.set_control(MotionMagicDutyCycle(PivotConstants.SCOREPOSDOWN))
