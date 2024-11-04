from commands2 import Command
import math
from typing import Callable
from wpilib import Timer
from wpilib.shuffleboard import BuiltInWidgets, Shuffleboard
from wpimath.controller import ProfiledPIDController
from wpimath.trajectory import TrapezoidProfile

from constants import *
from util import *
from subsystems.drive.drivetrain import Drivetrain

class DriveMaintainHeadingCommand(Command):

    heading_controller = ProfiledPIDController(
        2.5,
        0.0,
        0.1,
        TrapezoidProfile.Constraints(
            degs_to_rads(720), 
            degs_to_rads(1440)
        )
    )
    heading_controller.enableContinuousInput(-math.pi, math.pi)

    Shuffleboard.getTab("Tuning").add("Heading Controller", heading_controller).withWidget(BuiltInWidgets.kPIDController)

    heading_setpoint = None

    def __init__(self, drivetrain: DrivetrainConstants, throttle: Callable[[], float], strafe: Callable[[], float], turn: Callable[[], float]) -> None:
        """Drives the robot with field-centric steering.
        Uses a PID controller to maintain a steady heading in case of collisions or drift.

        All magnitudes must be between -1 and 1 (inclusive).

        Args:
            drivetrain (Drivetrain): Drivetrain instance.
            throttle (float): Magnitude of speed X (forward).
            strafe (float): Magnitude of speed Y (sideways).
            turn (float): Magnitude of rotation.
        """

        self.drivetrain = drivetrain
        self.throttle = throttle
        self.strafe = strafe
        self.turn = turn

        self.joystick_last_touched = -1.0
        self.heading_setpoint = None

        self.addRequirements(drivetrain)
        self.setName("Drive (Maintain Heading)")
        
    def initialize(self):
        self.heading_setpoint = None
        self.heading_controller.reset(self.drivetrain.get_yaw().radians())

    def execute(self):
        throttle = clamp(self.throttle(), -1, 1) * DrivetrainConstants.k_max_drive_speed
        strafe = clamp(self.strafe(), -1, 1) * DrivetrainConstants.k_max_drive_speed
        turn = clamp(self.turn(), -1, 1)

        if turn != 0:
            self.joystick_last_touched = Timer.getFPGATimestamp()
            
        if turn != 0 or Timer.getFPGATimestamp() - self.joystick_last_touched <= 0.25 and math.fabs(degs_to_rads(self.drivetrain.get_yaw_rate())) >= degs_to_rads(10):
            turn *= DrivetrainConstants.k_max_rot_rate
            self.heading_setpoint = None
            self.heading_controller.reset(self.drivetrain.get_yaw().radians())
        else: # If we haven't tried rotating in a bit, begin to maintain heading
            if self.heading_setpoint is None:
                self.heading_setpoint = self.drivetrain.get_yaw().radians()

            turn = self.heading_controller.calculate(self.drivetrain.get_yaw().radians(), self.heading_setpoint)


        self.drivetrain.drive(
            throttle,
            strafe,
            turn
        )

    def isFinished(self):
        return False
