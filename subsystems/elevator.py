import math
from enum import Enum

from commands2 import Command
from commands2.sysid import SysIdRoutine
from phoenix6 import SignalLogger, BaseStatusSignal, utils
from phoenix6.configs import TalonFXConfiguration, MotorOutputConfigs, FeedbackConfigs
from phoenix6.configs.config_groups import NeutralModeValue, MotionMagicConfigs, InvertedValue, DifferentialSensorSourceValue, DifferentialSensorsConfigs
from phoenix6.controls import Follower, VoltageOut, DynamicMotionMagicVoltage, CoastOut, MotionMagicVoltage
from phoenix6.hardware import TalonFX
from wpilib.sysid import SysIdRoutineLog
from wpimath.geometry import Pose3d, Rotation3d
from wpilib import DataLogManager

from constants import Constants
from subsystems import StateSubsystem


class ElevatorSubsystem(StateSubsystem):
    """
    The ElevatorSubsystem is responsible for controlling the elevator mechanism.
    """

    class SubsystemState(Enum):
        IDLE = None
        DEFAULT = Constants.ElevatorConstants.DEFAULT_POSITION
        L1 = Constants.ElevatorConstants.L1_SCORE_POSITION
        L2 = Constants.ElevatorConstants.L2_SCORE_POSITION
        L3 = Constants.ElevatorConstants.L3_SCORE_POSITION
        ALGAE_REEF = Constants.ElevatorConstants.L2_ALGAE_POSITION
        ALGAE_GROUND = Constants.ElevatorConstants.L3_ALGAE_POSITION
        PROCESSOR = Constants.ElevatorConstants.PROCESSOR_SCORE_POSITION

    _motor_config = (TalonFXConfiguration()
                     .with_slot0(Constants.ElevatorConstants.GAINS)
                     .with_motor_output(MotorOutputConfigs().with_neutral_mode(NeutralModeValue.BRAKE).with_inverted(InvertedValue.CLOCKWISE_POSITIVE))
                     .with_feedback(FeedbackConfigs().with_sensor_to_mechanism_ratio(Constants.ElevatorConstants.GEAR_RATIO))
                     .with_motion_magic(
        MotionMagicConfigs()
        .with_motion_magic_acceleration(Constants.ElevatorConstants.MM_DOWNWARD_ACCELERATION)
        .with_motion_magic_cruise_velocity(Constants.ElevatorConstants.CRUISE_VELOCITY)
        # .with_motion_magic_expo_k_v(Constants.ElevatorConstants.EXPO_K_V)
        # .with_motion_magic_expo_k_a(Constants.ElevatorConstants.EXPO_K_A)
        )
                     )

    def __init__(self) -> None:
        super().__init__("Elevator", self.SubsystemState.DEFAULT)

        self._master_motor = TalonFX(Constants.CanIDs.RIGHT_ELEVATOR_TALON) # Right Motor
        self._master_motor.configurator.apply(self._motor_config)

        self.follower_motor = TalonFX(Constants.CanIDs.LEFT_ELEVATOR_TALON) # Left Motor
        follower_config = TalonFXConfiguration()
        follower_config.slot0 = Constants.ElevatorConstants.GAINS
        follower_config.motor_output.with_neutral_mode(NeutralModeValue.BRAKE)
        follower_config.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        follower_config.feedback.with_sensor_to_mechanism_ratio(Constants.ElevatorConstants.GEAR_RATIO)
        follower_config.with_motion_magic(
            MotionMagicConfigs()
            .with_motion_magic_acceleration(Constants.ElevatorConstants.MM_DOWNWARD_ACCELERATION)
            .with_motion_magic_cruise_velocity(Constants.ElevatorConstants.CRUISE_VELOCITY)
        )

        self.follower_motor.configurator.apply(follower_config)
        
        # Use Follower control to make the follower motor follow the master
        self.follower_motor.set_control(Follower(self._master_motor.device_id, True))

        # Use simpler MotionMagicVoltage instead of DynamicMotionMagicVoltage

        self._position_request = MotionMagicVoltage(0)
        self._brake_request = MotionMagicVoltage(0)

        self._sys_id_request = VoltageOut(0)
        self._sys_id_routine = SysIdRoutine(
            SysIdRoutine.Config(
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdElevator_State", SysIdRoutineLog.stateEnumToString(state)
                )  # Log to .hoot for ease of access
            ),
            SysIdRoutine.Mechanism(
                lambda output: self._master_motor.set_control(self._sys_id_request.with_output(output)),
                lambda log: None,
                self,
            )
        )
        self._master_motor.set_position(0)


    def periodic(self) -> None:
        super().periodic()

        latency_compensated_position = BaseStatusSignal.get_latency_compensated_value(
            self._master_motor.get_position(), self._master_motor.get_velocity()
        )
        self._at_setpoint = abs(latency_compensated_position - self._position_request.position) <= Constants.ElevatorConstants.SETPOINT_TOLERANCE
        self.get_network_table().getEntry("At Setpoint").setBoolean(self._at_setpoint)
        
        # Log position and velocity for debugging
        self.get_network_table().getEntry("Position").setDouble(latency_compensated_position)
        self.get_network_table().getEntry("Velocity").setDouble(self._master_motor.get_velocity().value)
        self.get_network_table().getEntry("Target Position").setDouble(self._position_request.position)

    def set_desired_state(self, desired_state: SubsystemState) -> None:
        if not super().set_desired_state(desired_state):
            DataLogManager.log(f"Elevator: State change rejected - current state is {self._subsystem_state}, desired state is {desired_state}")
            return

        DataLogManager.log(f"Elevator: Changing state to {desired_state} with position {desired_state.value}")
        position = desired_state.value

        if position is None:
            DataLogManager.log("Elevator: Setting brake request")
            self._brake_request.position = self._master_motor.get_position().value
            self._master_motor.set_control(self._brake_request)
        else:
            current_pos = self._master_motor.get_position().value
            DataLogManager.log(f"Elevator: Moving from {current_pos} to {position}")
            
            self._position_request.position = position
            DataLogManager.log(f"Elevator: Setting position request to {position}")
            self._master_motor.set_control(self._position_request)
    

    def is_at_setpoint(self) -> bool:
        if self._subsystem_state is self.SubsystemState.IDLE:
            return False
        latency_compensated_position = BaseStatusSignal.get_latency_compensated_value(
            self._master_motor.get_position(), self._master_motor.get_velocity()
        )
        self._at_setpoint = abs(latency_compensated_position - self._position_request.position) <= Constants.ElevatorConstants.SETPOINT_TOLERANCE
        self.get_network_table().getEntry("At Setpoint").setBoolean(self._at_setpoint)
        return self._at_setpoint

    def stop(self) -> Command:
        return self.runOnce(lambda: self._master_motor.set_control(self._brake_request))

    def get_height(self) -> float:
        """Returns the height of the elevator, in meters."""
        return (self._master_motor.get_position().value / Constants.ElevatorConstants.GEAR_RATIO) * (2 * math.pi * 0.508)
    
    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        return self._sys_id_routine.quasistatic(direction).andThen(self.stop())

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        return self._sys_id_routine.dynamic(direction).andThen(self.stop())

    def get_component_poses(self) -> tuple[Pose3d, Pose3d]:
        position = self._master_motor.get_position().value
        return (
            Pose3d(0, 0, (position * (0.6985 / 6.096924)), Rotation3d()),
            Pose3d(0, 0, (position * 2 * (0.6985 / 6.096924)), Rotation3d())
        )

    def get_target_poses(self) -> tuple[Pose3d, Pose3d]:
        reference = self._master_motor.get_closed_loop_reference().value
        return (
            Pose3d(0, 0, (reference * (0.6985 / 6.096924)), Rotation3d()),
            Pose3d(0, 0, (reference * 2 * (0.6985 / 6.096924)), Rotation3d())
        )