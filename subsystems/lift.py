from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import NeutralModeValue
from phoenix6.controls import DutyCycleOut, Follower, MotionMagicDutyCycle
from phoenix6.hardware import TalonFX
from commands2 import Subsystem
from constants import *
from enum import Enum
from wpilib import Timer

class LiftStates(Enum):
    LOWERED = 0
    RAISED = 1
    CONTROLLED = 2
    STOPPED = 3

class Lift(Subsystem):
    
    def __init__(self):
        super().__init__()
        self.setName("Lift")
        
        self.master_motor = TalonFX(MotorIDs.LIFTMOTOR_RIGHT) # Right Motor
        self.follower_motor = TalonFX(MotorIDs.LIFTMOTOR_LEFT) # Left Motor
        elevator_config = TalonFXConfiguration()
        elevator_config.slot0.with_k_p(1)
        elevator_config.motor_output.with_neutral_mode(NeutralModeValue.BRAKE)
        elevator_config.current_limits.with_supply_current_limit_enable(True).with_supply_current_limit(LiftConstants.CURRENTSUPPLYLIMIT)
        elevator_config.motion_magic.with_motion_magic_cruise_velocity(LiftConstants.MM_VEL).with_motion_magic_acceleration(LiftConstants.MM_ACCEL)
        self.master_motor.configurator.apply(elevator_config)
        self.follower_motor.configurator.apply(elevator_config)

        self.master_motor.set_position(LiftConstants.TOPPOSITION)
        self.follower_motor.set_control(Follower(self.master_motor.device_id, False))
        
        self.status_timer = Timer()
        self.state = LiftStates.RAISED
        
    def getState(self) -> LiftStates:
        return self.state
        
    def setControl(self, control) -> None:
        self.master_motor.set_control(control)
        self.state = LiftStates.CONTROLLED
    
    def stop(self) -> None:
        self.master_motor.set_control(DutyCycleOut(0))
        self.state = LiftStates.STOPPED
         
    def extend(self) -> None:
        self.master_motor.set_control(MotionMagicDutyCycle(LiftConstants.TOPPOSITION))
        self.state = LiftStates.RAISED

    def compress(self) -> None:
        self.master_motor.set_control(MotionMagicDutyCycle(LiftConstants.BOTTOMPOSITION))
        self.state = LiftStates.LOWERED
