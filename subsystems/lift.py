from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import DifferentialSensorSourceValue, DifferentialSensorsConfigs, NeutralModeValue
from phoenix6.controls import CoastOut, DifferentialFollower, DutyCycleOut, PositionDutyCycle
from phoenix6.hardware import TalonFX
from commands2 import Subsystem
from constants import *
from enum import Enum

class LiftStates(Enum):
    LOWERED = 0
    RAISED = 1
    CONTROLLED = 2
    STOPPED = 3
    SCORE = 4

class Lift(Subsystem):
    
    def __init__(self):
        super().__init__()
        self.setName("Lift")
        
        self.master_motor = TalonFX(MotorIDs.LIFTMOTOR_RIGHT) # Right Motor
        general_config = TalonFXConfiguration()
        general_config.slot0.with_k_p(LiftConstants.K_P).with_k_i(LiftConstants.K_I)
        general_config.motor_output.with_neutral_mode(NeutralModeValue.BRAKE)
        general_config.current_limits.with_supply_current_limit_enable(True).with_supply_current_limit(LiftConstants.CURRENTSUPPLYLIMIT)
        general_config.motion_magic.with_motion_magic_cruise_velocity(LiftConstants.MM_VEL).with_motion_magic_acceleration(LiftConstants.MM_ACCEL)
        self.master_motor.configurator.apply(general_config)
        
        self.master_motor.set_position(LiftConstants.TOPPOSITION)
        
        
        self.follower_motor = TalonFX(MotorIDs.LIFTMOTOR_LEFT) # Left Motor
        follower_config = general_config
        follower_config.with_differential_sensors(
            DifferentialSensorsConfigs()
            .with_differential_sensor_source(DifferentialSensorSourceValue.REMOTE_TALON_FX_DIFF)
            .with_differential_talon_fx_sensor_id(self.master_motor.device_id)
            .with_differential_remote_sensor_id(self.master_motor.device_id)
        )
        self.follower_motor.configurator.apply(follower_config)
        
        self.follower_motor.set_control(CoastOut())
        
        self.state = LiftStates.RAISED
        
    def getState(self) -> LiftStates:
        return self.state
    
    def activateFollower(self) -> None:
        self.follower_motor.set_control(DifferentialFollower(self.master_motor.device_id, False))
    
    def disableFollower(self) -> None:
        self.follower_motor.set_control(CoastOut())
        
    def setControl(self, control) -> None:
        self.master_motor.set_control(control)
        self.state = LiftStates.CONTROLLED
    
    def stop(self) -> None:
        self.master_motor.set_control(DutyCycleOut(0))
        self.state = LiftStates.STOPPED
         
    def raiseFull(self) -> None:
        self.master_motor.set_control(PositionDutyCycle(LiftConstants.TOPPOSITION))
        self.state = LiftStates.RAISED

    def compressFull(self) -> None:
        self.master_motor.set_control(PositionDutyCycle(LiftConstants.BOTTOMPOSITION))
        self.state = LiftStates.LOWERED

    def scoreShoot(self) -> None:
        self.master_motor.set_control(PositionDutyCycle(LiftConstants.SCOREPOSITION))
        self.state = LiftStates.SCORE
