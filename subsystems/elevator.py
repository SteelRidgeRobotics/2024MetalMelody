from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import NeutralModeValue
from phoenix6.controls import DutyCycleOut, Follower, MotionMagicDutyCycle
from phoenix6.hardware import TalonFX
from commands2 import Subsystem
from constants import *
from enum import Enum
from wpilib import Timer

class ElevatorStates(Enum):
    LOWERED = 0
    RAISED = 1
    CONTROLLED = 2
    CALIBRATED = 3

class Elevator(Subsystem):
    
    def __init__(self):
        super().__init__()
        self.master_motor = TalonFX(MotorIDs.ELEVATORMOTOR_RIGHT) # Right Motor
        self.follower_motor = TalonFX(MotorIDs.ELEVATORMOTOR_LEFT) # Left Motor
        elevator_config = TalonFXConfiguration()
        elevator_config.slot0.with_k_p(1)
        elevator_config.motor_output.with_neutral_mode(NeutralModeValue.BRAKE)
        elevator_config.current_limits.with_supply_current_limit_enable(True).with_supply_current_limit(ElevatorConstants.CURRENTSUPPLYLIMIT)
        elevator_config.motion_magic.with_motion_magic_cruise_velocity(ElevatorConstants.MM_VEL).with_motion_magic_acceleration(ElevatorConstants.MM_ACCEL)
        self.master_motor.configurator.apply(elevator_config)
        self.follower_motor.configurator.apply(elevator_config)

        self.master_motor.set_position(ElevatorConstants.TOPPOSITION)
        self.follower_motor.set_control(Follower(self.master_motor.device_id, False))
        
        self.status_timer = Timer()
        self.state = ElevatorStates.RAISED
        
    """    
    def periodic(self) -> None:
        if self.state == ElevatorStates.LOWERED and self.master_motor.get_rotor_velocity().value == 0:
            self.status_timer.start()
        else:
            self.status_timer.reset()
            self.status_timer.stop()
            
        if self.status_timer.get() >= 0.25:
            self.master_motor.set_position(ElevatorConstants.BOTTOMPOSITION)
            self.state = ElevatorStates.CALIBRATED
    """
        
    def setDutyCycle(self, duty_cycle: DutyCycleOut) -> None:
        self.master_motor.set_control(duty_cycle)
        self.state = ElevatorStates.CONTROLLED
         
    def up(self) -> None:
        self.master_motor.set_control(MotionMagicDutyCycle(ElevatorConstants.TOPPOSITION))
        self.state = ElevatorStates.RAISED

    def below(self) -> None:
        self.master_motor.set_control(MotionMagicDutyCycle(ElevatorConstants.BOTTOMPOSITION))
        self.state = ElevatorStates.LOWERED
