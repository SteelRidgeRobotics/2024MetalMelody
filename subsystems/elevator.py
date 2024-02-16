from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import Follower, MotionMagicDutyCycle
from phoenix6.hardware import TalonFX
from commands2 import Subsystem
from constants import * 

class Elevator(Subsystem):
    
    def __init__(self):
        super().__init__()
        self.elevatorMotorRight = TalonFX(MotorIDs.ELEVATORMOTOR_RIGHT)
        self.elevatorMotorLeft = TalonFX(MotorIDs.ELEVATORMOTOR_LEFT)
        elevator_config = TalonFXConfiguration()
        elevator_config.slot0.with_k_p(ElevatorConstants.kP)
        elevator_config.motion_magic.with_motion_magic_acceleration(1).with_motion_magic_cruise_velocity(ElevatorConstants.MOTIONMAGICVELOCITY).with_motion_magic_jerk(ElevatorConstants.MOTIONMAGICJERK)
        elevator_config.current_limits.with_supply_current_limit_enable(True).with_supply_current_limit(ElevatorConstants.CURRENTSUPPLYLIMIT)
        self.elevatorMotorRight.configurator.apply(elevator_config)
        self.elevatorMotorLeft.configurator.apply(elevator_config)
        
        self.elevatorMotorLeft.set_position(0)
        self.elevatorMotorRight.set_position(0)
        self.elevatorMotorLeft.set_control(Follower(self.elevatorMotorRight.device_id, True))
        
    def up(self) -> None:
        self.elevatorMotorRight.set_control(MotionMagicDutyCycle(ElevatorConstants.TOPPOSITION))
        
    def below(self) -> None:
        self.elevatorMotorRight.set_control(MotionMagicDutyCycle(ElevatorConstants.BOTTOMPOSITION))
