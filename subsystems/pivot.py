from commands2 import Subsystem
from enum import Enum
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import NeutralModeValue
from phoenix6.controls import MotionMagicDutyCycle
from phoenix6.hardware import TalonFX
from constants import *

class PivotStates(Enum):
    STOWED = 0
    SCORE_UP = 1
    SCORE_DOWN = 2
    INTAKE = 3

class Pivot(Subsystem):
    
    def __init__(self) -> None:
        super().__init__()
        self.setName("Pivot")
        
        self.pivotMotor = TalonFX(MotorIDs.PIVOTMOTOR)
        pivot_config = TalonFXConfiguration()
        pivot_config.motor_output.with_neutral_mode(NeutralModeValue.BRAKE).with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
        pivot_config.slot0.with_k_p(PivotConstants.K_P).with_k_i(PivotConstants.K_I).with_k_d(PivotConstants.K_D).with_k_v(PivotConstants.K_V).with_k_s(PivotConstants.K_S)
        pivot_config.feedback.with_sensor_to_mechanism_ratio(PivotConstants.GEAR_RATIO)
        pivot_config.motion_magic.with_motion_magic_acceleration(PivotConstants.MM_ACCELERATION).with_motion_magic_cruise_velocity(PivotConstants.MM_CRUISE_VEL).with_motion_magic_jerk(PivotConstants.MM_JERK)
        pivot_config.current_limits.with_supply_current_limit(PivotConstants.SUPPLY_LIMIT).with_supply_current_limit_enable(True)
        self.pivotMotor.configurator.apply(pivot_config)
        
        self.pivotMotor.set_position(0)

        self.state = PivotStates.STOWED

    def getState(self) -> PivotStates:
        return self.state
        
    def intake(self) -> None:
        self.pivotMotor.set_control(MotionMagicDutyCycle(PivotConstants.INTAKEPOS))
        self.state = PivotStates.INTAKE

    def stow(self) -> None:
        self.pivotMotor.set_control(MotionMagicDutyCycle(PivotConstants.STOWPOS))
        self.state = PivotStates.STOWED

    def scoreUpwards(self) -> None:
        self.pivotMotor.set_control(MotionMagicDutyCycle(PivotConstants.SCOREPOSUP))
        self.state = PivotStates.SCORE_UP

    def scoreDownwards(self) -> None:
        self.pivotMotor.set_control(MotionMagicDutyCycle(PivotConstants.SCOREPOSDOWN))
        self.state = PivotStates.SCORE_DOWN
