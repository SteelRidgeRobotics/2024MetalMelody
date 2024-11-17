from ntcore import NetworkTableInstance
from phoenix6 import SignalLogger, swerve, units
from wpilib import DataLogManager, DriverStation, Field2d, SmartDashboard
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState

class Telemetry:
    def __init__(self, max_speed: units.meters_per_second):
        """
        Construct a telemetry object with the specified max speed of the robot.

        :param max_speed: Maximum speed
        :type max_speed: units.meters_per_second
        """
        self._max_speed = max_speed
        SignalLogger.start()
        DriverStation.startDataLog(DataLogManager.getLog())

        self._field = Field2d()
        SmartDashboard.putData("Field", self._field)

        # Robot speeds for general checking
        self._table = NetworkTableInstance.getDefault().getTable("Telemetry")
        self._current_pose = self._table.getStructTopic("currentPose", Pose2d).publish()
        self._chassis_speeds = self._table.getStructTopic("chassisSpeeds", ChassisSpeeds).publish()
        self._odom_freq = self._table.getDoubleTopic("Odometry Frequency").publish()

        # Additional swerve info
        self._module_states = self._table.getStructArrayTopic("moduleStates", SwerveModuleState).publish()
        self._module_targets = self._table.getStructArrayTopic("moduleTargets", SwerveModuleState).publish()

    def telemeterize(self, state: swerve.SwerveDrivetrain.SwerveDriveState):
        """
        Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger.
        """

        self._field.setRobotPose(state.pose)
        self._current_pose.set(state.pose)

        self._odom_freq.set(1.0 / state.odometry_period)

        self._module_states.set(state.module_states)
        self._module_targets.set(state.module_targets)
        self._chassis_speeds.set(state.speeds)
