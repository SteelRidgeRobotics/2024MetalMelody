from phoenix6 import BaseStatusSignal
from phoenix6.configs.talon_fx_configs import InvertedValue, NeutralModeValue, TalonFXConfiguration
from phoenix6.hardware.talon_fx import TalonFX

class MotorIDs:
    LEFT_FRONT_DRIVE = 1
    LEFT_REAR_DRIVE = 2
    RIGHT_FRONT_DRIVE = 3
    RIGHT_REAR_DRIVE = 4

    LEFT_FRONT_DIRECTION = 5
    LEFT_REAR_DIRECTION = 6
    RIGHT_FRONT_DIRECTION = 7
    RIGHT_REAR_DIRECTION = 8
    
    PIVOTMOTOR = 9
    
    INTAKEMOTOR = 10
    
    ELEVATORMOTOR_RIGHT = 11
    ELEVATORMOTOR_LEFT = 12
    
class CANIDs:
    LEFT_FRONT = 5
    LEFT_REAR = 6
    RIGHT_FRONT = 7
    RIGHT_REAR = 8

class IntakeConstants:
    INTAKESPEED = .1875 * 4

class PivotConstants:
    MM_ACCELERATION = 2
    MM_CRUISE_VEL = 2
    STOWPOS = 0.5
    INTAKEPOS = 0.1487
    SCOREPOS = 0.194
    K_P = 10
    K_I = 0
    K_D = 0.2
    GEAR_RATIO = 48

class ElevatorConstants:
    CURRENTSUPPLYLIMIT = 4
    TOPPOSITION = 123
    BOTTOMPOSITION = 0
    MM_ACCEL = 100
    MM_VEL = 100
    CLIMB_MM_ACCEL = 10
    CLIMB_MM_VEL = 25

class LimelightConstants:
    RESOLUTIONX = 1280
    RESOLUTIONY = 960

class ExternalConstants:
    DRIVERCONTROLLER = 0
    DEADBAND = 0.15
    FUNCTIONSCONTROLLER = 1
    
class DriveConstants:
    rotation_kP = 0.1
    translation_kP = 0.25
    
"""
SWERVE
    """
    
class SwerveConstants:
    k_wheel_size = 0.1 # meters
    k_max_module_speed = 4 # m/s
    k_max_rot_rate = 10 # rad/s
    k_drive_base_radius = 0.43 # meters
    auto_kP_translation = 1.5
    auto_kP_rotation = 2.5

class DriveMotorConstants:

    def __init__(self, motor_id: int, 
                 k_s: float=0.0, k_v: float=0, k_a: float=0, k_p: float=0.14, k_i: float=0, k_d: float=0, inverted: InvertedValue=InvertedValue.COUNTER_CLOCKWISE_POSITIVE) -> None:
        
        self.motor_id = motor_id
        
        self.k_s = k_s
        self.k_v = k_v
        self.k_a = k_a
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        
        self.inverted = inverted
        
        self.neutral_mode = NeutralModeValue.COAST
        
    def apply_configuration(self, motor: TalonFX) -> TalonFX:
        config = TalonFXConfiguration()
        config.slot0.with_k_s(self.k_s).with_k_v(self.k_v).with_k_a(self.k_a).with_k_p(self.k_p).with_k_i(self.k_i).with_k_d(self.k_d)
        config.motor_output.with_neutral_mode(self.neutral_mode).with_inverted(self.inverted)
        config.feedback.sensor_to_mechanism_ratio = k_drive_gear_ratio
        motor.configurator.apply(config)
        
        return motor
        
class DirectionMotorConstants:
    
    def __init__(self, motor_id: int, 
                 k_s: float=0.26, cruise_velocity: int=240, cruise_acceleration: int=600, cruise_jerk: int=6500, 
                 k_v: float=0.1186, k_a: float=0, k_p: float=6, k_i: float=0.2, k_d: float=0) -> None:
        
        self.motor_id = motor_id
        
        self.k_s = k_s
        self.k_v = k_v
        self.k_a = k_a
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        
        self.cruise_velocity = cruise_velocity
        self.cruise_acceleration = cruise_acceleration
        self.cruise_jerk = cruise_jerk
        
        self.peak_volt = 10
        
        self.neutral_mode = NeutralModeValue.BRAKE
        self.invert = InvertedValue.CLOCKWISE_POSITIVE
        
    def apply_configuration(self, motor: TalonFX) -> TalonFX:
        config = TalonFXConfiguration()
        config.slot0.with_k_s(self.k_s).with_k_v(self.k_v).with_k_a(self.k_a).with_k_p(self.k_p).with_k_i(self.k_i).with_k_d(self.k_d)
        config.motor_output.with_neutral_mode(self.neutral_mode).with_inverted(self.invert)
        config.voltage.with_peak_forward_voltage(self.peak_volt).with_peak_reverse_voltage(-self.peak_volt)
        config.motion_magic.with_motion_magic_cruise_velocity(self.cruise_velocity).with_motion_magic_acceleration(self.cruise_acceleration).with_motion_magic_jerk(self.cruise_jerk)
        motor.configurator.apply(config)
        return motor
    
k_direction_gear_ratio = 150 / 7
k_drive_gear_ratio = 27 / 4
