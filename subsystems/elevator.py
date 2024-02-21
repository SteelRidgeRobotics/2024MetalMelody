from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import NeutralModeValue
from phoenix6.controls import DynamicMotionMagicDutyCycle, Follower, MotionMagicDutyCycle
from phoenix6.hardware import TalonFX
from commands2 import Subsystem
from constants import *

class Elevator(Subsystem):
    
    def __init__(self):
        super().__init__()
        self.master_motor = TalonFX(MotorIDs.ELEVATORMOTOR_RIGHT) # Right Motor
        self.follower_motor = TalonFX(MotorIDs.ELEVATORMOTOR_LEFT) # Left Motor
        elevator_config = TalonFXConfiguration()
        elevator_config.slot0.with_k_p(1)
        elevator_config.motor_output.with_neutral_mode(NeutralModeValue.BRAKE)
        elevator_config.current_limits.with_supply_current_limit_enable(True).with_supply_current_limit(ElevatorConstants.CURRENTSUPPLYLIMIT)
        self.master_motor.configurator.apply(elevator_config)
        self.follower_motor.configurator.apply(elevator_config)

        self.master_motor.set_position(ElevatorConstants.TOPPOSITION)
        self.follower_motor.set_control(Follower(self.master_motor.device_id, True))
         
    def up(self) -> None:
        self.master_motor.set_control(DynamicMotionMagicDutyCycle(ElevatorConstants.TOPPOSITION, ElevatorConstants.MM_VEL, ElevatorConstants.MM_ACCEL, 0))

    def robotDown(self) -> None:
        self.master_motor.set_control(DynamicMotionMagicDutyCycle(ElevatorConstants.TOPPOSITION, ElevatorConstants.CLIMB_MM_VEL, ElevatorConstants.CLIMB_MM_ACCEL, 0))

    def below(self) -> None:
        self.master_motor.set_control(DynamicMotionMagicDutyCycle(ElevatorConstants.BOTTOMPOSITION, ElevatorConstants.MM_VEL, ElevatorConstants.MM_ACCEL, 0))

    def robotUp(self) -> None:
        self.master_motor.set_control(DynamicMotionMagicDutyCycle(ElevatorConstants.BOTTOMPOSITION, ElevatorConstants.CLIMB_MM_VEL, ElevatorConstants.CLIMB_MM_ACCEL, 0))
