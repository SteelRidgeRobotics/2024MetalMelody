from commands2 import Subsystem
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration
from phoenix6.configs.cancoder_configs import AbsoluteSensorRangeValue
from phoenix6.configs.config_groups import *
from phoenix6.controls import VelocityVoltage, PositionVoltage
from phoenix6.hardware import CANcoder, ParentDevice, TalonFX
from phoenix6.sim import ChassisReference
from phoenix6.status_signal import BaseStatusSignal
from phoenix6 import unmanaged
from wpilib import DriverStation, RobotBase, RobotController, SmartDashboard
from wpilib.simulation import DCMotorSim
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpimath.system.plant import DCMotor

from constants import *
from util import *

class SwerveModule(Subsystem):
    """A single module on the drive train, comprised of 2 motors: one for driving, one for steering."""
    
    # Creating configs for each device
    
    ### Drive motor ###
    drive_config = TalonFXConfiguration()
    
    # Tuning
    drive_config.slot0 = DrivetrainConstants.k_drive_gains
    
    # Slip Current
    drive_config.current_limits.stator_current_limit_enable = True
    drive_config.current_limits.stator_current_limit = DrivetrainConstants.k_slip_current
    
    drive_config.motor_output.neutral_mode = NeutralModeValue.COAST
    
    # Feedback
    drive_config.feedback.sensor_to_mechanism_ratio = DrivetrainConstants.k_drive_gear_ratio
    
    ### Steer motor ###
    steer_config = TalonFXConfiguration()
    
    # Tuning
    steer_config.slot0 = DrivetrainConstants.k_steer_gains
    
    # Sensors and Feedback
    steer_config.feedback.feedback_sensor_source = FeedbackSensorSourceValue.FUSED_CANCODER

    steer_config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
    steer_config.motor_output.neutral_mode = NeutralModeValue.BRAKE
    
    steer_config.closed_loop_general.continuous_wrap = True # This does our angle optimizations for us (yay)
    
    steer_config.feedback.rotor_to_sensor_ratio = DrivetrainConstants.k_steer_gear_ratio
    
    
    ### Encoder ###
    encoder_config = CANcoderConfiguration()
    
    # Sensor range (basically makes our offset be between [-0.5, 0.5))
    encoder_config.magnet_sensor.absolute_sensor_range = AbsoluteSensorRangeValue.SIGNED_PLUS_MINUS_HALF
    
    
    def __init__(self, drive_id: int, steer_id: int, encoder_id: int, encoder_offset: float) -> None:
        
        # Finish configuring
        
        # Steer motors (set sensor id)
        module_steer_config = self.steer_config # (don't edit static configs! Move them into a temp variable and edit that)
        module_steer_config.feedback.feedback_remote_sensor_id = encoder_id # Bind the encoder to the talon

        # Encoder configs (set magnet offset)
        module_encoder_config = self.encoder_config
        module_encoder_config.magnet_sensor.magnet_offset = encoder_offset
        
        
        # Create CAN devices and apply the new configs
        
        self.drive_talon = TalonFX(drive_id, DrivetrainConstants.k_canbus_name)
        self.drive_talon.configurator.apply(self.drive_config)
        
        self.steer_talon = TalonFX(steer_id, DrivetrainConstants.k_canbus_name)
        self.steer_talon.configurator.apply(module_steer_config)
        
        self.encoder = CANcoder(encoder_id, DrivetrainConstants.k_canbus_name)
        self.encoder.configurator.apply(module_encoder_config)
        self.encoder.sim_state.orientation = ChassisReference.Clockwise_Positive
        
        # Create control requests and set them to the talons
        self.steer_request = PositionVoltage(0)
        self.drive_request = VelocityVoltage(0)
        
        self.steer_talon.set_control(self.steer_request)
        self.drive_talon.set_control(self.drive_request)

        # Simulation Models
        self.steer_sim_model = DCMotorSim(
            DCMotor.krakenX60FOC(1),
            DrivetrainConstants.k_steer_gear_ratio,
            0.001
        )

        self.drive_sim_model = DCMotorSim(
            DCMotor.krakenX60FOC(1),
            DrivetrainConstants.k_drive_gear_ratio,
            0.0001
        )
        
        BaseStatusSignal.set_update_frequency_for_all(
            100,
            self.drive_talon.get_acceleration(),
            self.drive_talon.get_position(),
            self.drive_talon.get_velocity(),
            self.steer_talon.get_acceleration(),
            self.steer_talon.get_position(),
            self.steer_talon.get_velocity(),
            self.encoder.get_position(),
            self.encoder.get_velocity(),
        )
        
        ParentDevice.optimize_bus_utilization_for_all(self.drive_talon, self.steer_talon, self.encoder)

        if RobotBase.isReal():
            self.desired_state = self.get_state()
        else:
            from random import randint
            random_state = SwerveModuleState(angle=Rotation2d.fromDegrees(randint(0, 359)))

            self.desired_state = random_state
            self.steer_sim_model.setState(random_state.angle.radians(), 0)
        
        self.previous_desired_angle = self.desired_state.angle

        self.set_desired_state(self.desired_state)
        
    def simulationPeriodic(self) -> None:

        if DriverStation.isEnabled():
            unmanaged.feed_enable(100)
        
        # Steer Model
        steer_sim = self.steer_talon.sim_state
        encoder_sim = self.encoder.sim_state
        steer_sim.set_supply_voltage(RobotController.getBatteryVoltage())
        encoder_sim.set_supply_voltage(RobotController.getBatteryVoltage())
        self.steer_sim_model.setInputVoltage(steer_sim.motor_voltage)
        self.steer_sim_model.update(0.02)

        steer_sim.set_raw_rotor_position(rads_to_rots(self.steer_sim_model.getAngularPosition()) * DrivetrainConstants.k_steer_gear_ratio)
        steer_sim.set_rotor_velocity(rads_to_rots(self.steer_sim_model.getAngularVelocity()) * DrivetrainConstants.k_steer_gear_ratio)
        encoder_sim.set_raw_position(rads_to_rots(self.steer_sim_model.getAngularPosition()))
        encoder_sim.set_velocity(rads_to_rots(self.steer_sim_model.getAngularVelocity()))

        # Drive Model
        drive_sim = self.drive_talon.sim_state
        drive_sim.set_supply_voltage(RobotController.getBatteryVoltage())
        self.drive_sim_model.setInputVoltage(drive_sim.motor_voltage)
        self.drive_sim_model.update(0.02)

        drive_sim.set_raw_rotor_position(rads_to_rots(self.drive_sim_model.getAngularPosition()) * DrivetrainConstants.k_drive_gear_ratio)
        drive_sim.set_rotor_velocity(rads_to_rots(self.drive_sim_model.getAngularVelocity()) * DrivetrainConstants.k_drive_gear_ratio)
        
    def get_angle(self) -> Rotation2d:
        """Returns the current angle of the wheel by converting the steer motor position into degrees."""

        compensated_position = BaseStatusSignal.get_latency_compensated_value(
            self.steer_talon.get_position().refresh(), # This function doesn't refresh the status signals, so we do it here
            self.steer_talon.get_velocity().refresh(), # Same for this signal
            0.02
        )

        return Rotation2d.fromDegrees(rots_to_degs(compensated_position))
    
    def get_speed(self) -> float:
        """Returns the module's current driving speed (m/s)."""
        
        # Compensate for acceleration
        compensated_speed = BaseStatusSignal.get_latency_compensated_value(
            self.drive_talon.get_velocity().refresh(),
            self.drive_talon.get_acceleration().refresh(),
            0.06
        )
        
        return rot_to_meters(compensated_speed)
        
    def get_position(self) -> SwerveModulePosition:
        """Returns the current position of the module; distance traveled and current angle."""
        
        # Compensate for velocity
        compensated_position = BaseStatusSignal.get_latency_compensated_value(
            self.drive_talon.get_position().refresh(),
            self.drive_talon.get_velocity().refresh()
        )
        
        distance = rot_to_meters(compensated_position)
        
        return SwerveModulePosition(
            distance, 
            self.get_angle()
        )
    
    def get_state(self) -> SwerveModuleState:
        """Returns the module's current state; current speed (m/s) and current angle."""

        return SwerveModuleState(self.get_speed(), self.get_angle())
    
    def get_target(self) -> SwerveModuleState:
        """Returns the module's desired state."""

        return self.desired_state
        
    def set_desired_state(self, state: SwerveModuleState) -> None:
        """Sets the motor control requests to the desired state."""

        # Angle optimizations and reverse velocity if needed
        state = SwerveModuleState.optimize(state, self.previous_desired_angle)
        self.desired_state = state

        self.steer_talon.set_control(
            self.steer_request.with_position(degs_to_rots(state.angle.degrees()))
        )
        
        self.drive_talon.set_control(
            self.drive_request.with_velocity(meters_to_rots(state.speed))
        )

        self.previous_desired_angle = state.angle

    def stop(self) -> None:
        """Stops the module from moving."""

        self.set_desired_state(
            SwerveModuleState(0, self.previous_desired_angle)
        )
        