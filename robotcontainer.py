#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import commands2.button
import commands2.cmd
from commands2 import SequentialCommandGroup, WaitCommand
from commands2.sysid import SysIdRoutine
from numpy.f2py.capi_maps import lcb_map

from constants import Constants
from generated.tuner_constants import TunerConstants
from subsystems.intake import Intake
from subsystems.leds import LedSubsystem
from subsystems.lift import Lift
from subsystems.pivot import Pivot
from subsystems.pivot import PivotStates
from subsystems.vision import VisionSubsystem
from robot_state import RobotState

import math
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathConstraints, PathPlannerPath
from phoenix6 import SignalLogger, swerve
from phoenix6.controls import DutyCycleOut
from phoenix6.swerve.utility.phoenix_pid_controller import PhoenixPIDController
from wpilib import DriverStation, SmartDashboard, XboxController
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians

from commands.manual_lift import ManualLift
from commands.intake_and_stow import IntakeAndStow
from commands.vibrate import VibrateController


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self._max_speed = (
            TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            1
        )  # 3/4 of a rotation per second max angular velocity

        self._driver_controller = commands2.button.CommandXboxController(0)
        self._function_controller = commands2.button.CommandXboxController(1)

        self._robot_state = RobotState(self._max_speed)

        self.drivetrain = TunerConstants.create_drivetrain()

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(
                self._max_angular_rate * 0.1
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._face = (
            swerve.requests.FieldCentricFacingAngle()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(
                self._max_angular_rate * 0.1
            )
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )
        self._face.heading_controller = PhoenixPIDController(18.749, 0.45773, 0)
        self._face.heading_controller.enableContinuousInput(-math.pi, math.pi)

        self.intake = Intake()
        self.leds = LedSubsystem()
        self.lift = Lift()
        self.pivot = Pivot()
        #self.vision = VisionSubsystem(self.drivetrain)

        # Path follower
        self._auto_chooser = AutoBuilder.buildAutoChooser("Auto Chooser")
        SmartDashboard.putData("Auto Mode", self._auto_chooser)

        # Configure the button bindings
        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        -self._driver_controller.getLeftY() * self._max_speed
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        -self._driver_controller.getLeftX() * self._max_speed
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        -self._driver_controller.getRightX() * self._max_angular_rate
                    )  # Drive counterclockwise with negative X (left)
                )
            )
        )

        self._driver_controller.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        self._driver_controller.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(-self._driver_controller.getLeftY(), -self._driver_controller.getLeftX())
                )
            )
        )
        
        # We can't test these until we get a Limelight onto MM and until PathPlanner Beta 5 releases :(
        self._driver_controller.x().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._face.with_velocity_x(
                    -self._driver_controller.getLeftY() * self._max_speed
                )
                .with_velocity_y(
                    -self._driver_controller.getLeftX() * self._max_speed
                )
                .with_target_direction(
                    # Gets the angle to our alliance's speaker
                    (Constants.k_apriltag_layout.getTagPose(4 if (DriverStation.getAlliance() or DriverStation.Alliance.kBlue) == DriverStation.Alliance.kRed else 7).toPose2d().translation() - self.drivetrain.get_state().pose.translation()).angle() + Rotation2d.fromDegrees(180)
                )
            )
        )
        
        self._driver_controller.y().whileTrue(
            (AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("ScoreAmp"), PathConstraints(1, 1, 1, 1, unlimited=False))
             .andThen(
                SequentialCommandGroup(
                    self.pivot.runOnce(lambda: self.pivot.scoreDownwards()),
                    WaitCommand(1),
                    self.intake.runOnce(lambda: self.intake.disencumber()),
                    WaitCommand(1),
                    self.intake.runOnce(lambda: self.intake.stop()).alongWith(self.pivot.runOnce(lambda: self.pivot.stow()))
                )
            ))
        ).onFalse(
            self.pivot.runOnce(lambda: self.pivot.stow())
        )

        # Run SysId routines when holding back/start and X/Y.
        # Note that each routine should be run exactly once in a single log.
        (self._driver_controller.back() & self._driver_controller.y()).onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward).onlyIf(lambda: not DriverStation.isFMSAttached())
        )
        (self._driver_controller.back() & self._driver_controller.x()).onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse).onlyIf(lambda: not DriverStation.isFMSAttached())
        )
        (self._driver_controller.start() & self._driver_controller.y()).onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward).onlyIf(lambda: not DriverStation.isFMSAttached())
        )
        (self._driver_controller.start() & self._driver_controller.x()).onTrue(commands2.InstantCommand(lambda: SignalLogger.start())).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse).onlyIf(lambda: not DriverStation.isFMSAttached())
        )

        # reset the field-centric heading on left bumper press
        self._driver_controller.leftBumper().onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )

        self._function_controller.y().onTrue(
            self.lift.runOnce(self.lift.raiseFull).alongWith(self.pivot.runOnce(self.pivot.scoreDownwards))
        )

        self._function_controller.x().onTrue(
            self.pivot.runOnce(self.pivot.stow).alongWith(self.intake.runOnce(self.intake.stop))
        )

        self._function_controller.b().whileTrue(
            ManualLift(self._function_controller, self.lift)
        )

        self._function_controller.a().onTrue(
            self.lift.runOnce(self.lift.compressFull).alongWith(self.pivot.runOnce(self.pivot.stow))
        )

        self._function_controller.leftBumper().onTrue(
            IntakeAndStow(self.intake, self.pivot)
                .andThen(VibrateController(self._driver_controller, XboxController.RumbleType.kBothRumble, 0.75))
                .alongWith(VibrateController(self._function_controller, XboxController.RumbleType.kBothRumble, 0.25))
        )

        self._function_controller.rightBumper().onTrue(
            self.pivot.runOnce(lambda: self.pivot.pivotMotor.set_control(DutyCycleOut(0.1)))
                .onlyIf(lambda: self.pivot.getState() is PivotStates.SCORE_UP)
                .alongWith(self.intake.runOnce(self.intake.disencumber))
        ).onFalse(
            self.intake.runOnce(self.intake.stop).alongWith(self.pivot.runOnce(self.pivot.stow))
        )

        self._function_controller.leftStick().onTrue(
            self.lift.runOnce(self.lift.scoreShoot).alongWith(self.pivot.runOnce(self.pivot.scoreUpwards))
        )

        self._function_controller.rightStick().onTrue(
            self.lift.runOnce(self.lift.raiseFull).alongWith(self.pivot.runOnce(self.pivot.stow))
        )

        self.drivetrain.register_telemetry(
            lambda state: self._robot_state.log_swerve_state(state)
        )

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        return self._auto_chooser.getSelected()
