from commands2 import Subsystem
from enum import Enum
from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import NeutralModeValue, InvertedValue
from phoenix6.controls import MotionMagicDutyCycle
from phoenix6.hardware import TalonFX
from constants import Constants

class PivotStates(Enum):
    STOWED = 0
    SCORE_UP = 1
    SCORE_DOWN = 2
    INTAKE = 3

class Pivot(Subsystem):
    
    def __init__(self) -> None:
        super().__init__()
        self.setName("Pivot")
        
        self.pivotMotor = TalonFX(Constants.CanIDs.k_pivot_motor)
        pivot_config = TalonFXConfiguration()
        pivot_config.motor_output.with_neutral_mode(NeutralModeValue.BRAKE).with_inverted(InvertedValue.CLOCKWISE_POSITIVE)
        pivot_config.slot0 = Constants.PivotConstants.k_gains
        pivot_config.feedback.with_sensor_to_mechanism_ratio(Constants.PivotConstants.k_gear_ratio)
        pivot_config.motion_magic.with_motion_magic_acceleration(Constants.PivotConstants.k_acceleration).with_motion_magic_cruise_velocity(Constants.PivotConstants.k_cruise_velocity).with_motion_magic_jerk(Constants.PivotConstants.k_jerk)
        pivot_config.current_limits.with_supply_current_limit(Constants.PivotConstants.k_supply_current).with_supply_current_limit_enable(True)
        self.pivotMotor.configurator.apply(pivot_config)
        
        self.pivotMotor.set_position(0)

        self.state = PivotStates.STOWED

    def getState(self) -> PivotStates:
        return self.state
        
    def intake(self) -> None:
        self.pivotMotor.set_control(MotionMagicDutyCycle(Constants.PivotConstants.k_intake_pos))
        self.state = PivotStates.INTAKE

    def stow(self) -> None:
        self.pivotMotor.set_control(MotionMagicDutyCycle(Constants.PivotConstants.k_stow_pos))
        self.state = PivotStates.STOWED

    def scoreUpwards(self) -> None:
        self.pivotMotor.set_control(MotionMagicDutyCycle(Constants.PivotConstants.k_score_up_pos))
        self.state = PivotStates.SCORE_UP

    def scoreDownwards(self) -> None:
        self.pivotMotor.set_control(MotionMagicDutyCycle(Constants.PivotConstants.k_score_down_pos))
        self.state = PivotStates.SCORE_DOWN
