import wpilib
from commands2.button import JoystickButton
from constants import *
# from subsystems.drive import SwerveDrive
from subsystems.intake import IntakeAndPivot
from subsystems.elevator import Elevator
from subsystems.swerve import Swerve
from commands.intake_commands import *
from commands.drive import DriveByController
from commands2 import InstantCommand
from wpilib import SendableChooser, SmartDashboard
from wpimath.geometry import Pose2d

class RobotContainer:
    
    def __init__(self):
        
        """Sendables!!!"""
        self.start_chooser = SendableChooser()
        self.start_chooser.setDefaultOption("Failsafe", Pose2d())
        
        self.start_chooser.onChange(lambda: self.swerve.reset_odometry(self.start_chooser.getSelected()))
        SmartDashboard.putData("Starting Position", self.start_chooser)
        
        self.auto_chooser = SendableChooser()
        self.auto_chooser.setDefaultOption("Failsafe", None)
        SmartDashboard.putData("Autonomous Select", self.auto_chooser)

        self.driverController = wpilib.XboxController(ExternalConstants.DRIVERCONTROLLER)
        self.functionsController = wpilib.XboxController(ExternalConstants.FUNCTIONSCONTROLLER)

        self.swerve: Swerve = Swerve() # This helps IntelliSense know that this is a Swerve object, not a Subsystem (it gets confused sometimes)
        self.intake = IntakeAndPivot()
        self.elevator = Elevator()
        
        self.swerve.setDefaultCommand(DriveByController(self.swerve, self.driverController))

        JoystickButton(self.functionsController, wpilib.XboxController.Button.kA).whileTrue(FeederTest(self.intake))
        JoystickButton(self.functionsController, wpilib.XboxController.Button.kY).whileTrue(FeederTestDrop(self.intake))
        JoystickButton(self.functionsController, wpilib.XboxController.Button.kB).whileTrue(FeederTestStop(self.intake))
        JoystickButton(self.functionsController, wpilib.XboxController.Button.kX).onTrue(InstantCommand(lambda: self.elevator.togglePosition()))
        JoystickButton(self.functionsController, wpilib.XboxController.Button.kB).onTrue(InstantCommand(lambda: MovePivot(self.intake)))
        
    def runSelectedAutoCommand(self) -> None:
        self.swerve.initialize().reset_yaw().reset_odometry(self.start_chooser.getSelected())
        # NOTE: THIS WILL CRASH UNTIL WE ADD A FAILSAFE AUTO. JUST SAYING
        self.auto_chooser.getSelected().schedule() # It's gray, it's just because getSelected() can return literally anything. This works trust lmao
        