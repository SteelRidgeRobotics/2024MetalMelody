from phoenix6.configs import TalonFXConfiguration
from phoenix6.configs.config_groups import DifferentialSensorSourceValue, DifferentialSensorsConfigs, NeutralModeValue
from phoenix6.controls import Follower, MotionMagicVoltage
from phoenix6.hardware import TalonFX
from commands2 import Subsystem
from constants import *

class Swivel(Subsystem):

    def __init__(self):
        super().__init__()
        self.setName("Lift")
        
        self.master_motor = TalonFX(MotorIDs.SWIVELMOTOR_RIGHT) # Right Motor
        general_config = TalonFXConfiguration()
        general_config.slot0.with_k_p(SwivelConstants.K_P).with_k_i(SwivelConstants.K_I)
        general_config.motor_output.with_neutral_mode(NeutralModeValue.BRAKE)
        general_config.current_limits.with_supply_current_limit_enable(True).with_supply_current_limit(SwivelConstants.SUPPLY_LIMIT)
        general_config.motion_magic.with_motion_magic_cruise_velocity(SwivelConstants.MM_CRUISE_VEL).with_motion_magic_acceleration(SwivelConstants.MM_ACCELERATION)
        self.master_motor.configurator.apply(general_config)
        
        self.motion_magic_control = MotionMagicVoltage(SwivelConstants.TRANSFERPOS)
        
        self.follower_motor = TalonFX(MotorIDs.SWIVELMOTOR_LEFT) # Left Motor
        follower_config = general_config
        # follower_config.with_differential_sensors(
        #     DifferentialSensorsConfigs()
        #     .with_differential_sensor_source(DifferentialSensorSourceValue.REMOTE_TALON_FX_DIFF)
        #     .with_differential_talon_fx_sensor_id(self.master_motor.device_id)
        #     .with_differential_remote_sensor_id(self.master_motor.device_id)
        # )
        self.follower_motor.configurator.apply(follower_config)
        
        self.follower_motor.set_control(Follower(MotorIDs.SWIVELMOTOR_RIGHT))

    def reposition(self, position=SwivelConstants.TRANSFERPOS):
        
        self.master_motor.set_control(self.motion_magic_control.with_position(position))

        
    