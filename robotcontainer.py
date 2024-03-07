from commands.drive import DriveByController
from commands.intake_and_stow import IntakeAndStow
from commands.manual_elevator import ManualElevator
from commands.reset_pivot import ResetPivot
from commands.vibrate import VibrateController
from commands2 import InstantCommand
from commands2.button import JoystickButton
from constants import *
from pathplannerlib.auto import NamedCommands, PathPlannerAuto
from subsystems.elevator import Elevator
from subsystems.intake import Intake
from subsystems.swerve import Swerve
from wpilib import SendableChooser, SmartDashboard, XboxController
from wpimath.geometry import Pose2d, Rotation2d

class RobotContainer:
    
    def __init__(self):
        self.swerve: Swerve = Swerve()
        self.elevator: Elevator = Elevator()
        self.intake = Intake()
        self.swerve.initialize()
                
        # PathPlanner Commands
        ## Elevator
        
        NamedCommands.registerCommand("elevatorUp", InstantCommand(lambda: self.elevator.up(), self.elevator))
        NamedCommands.registerCommand("elevatorBelow", InstantCommand(lambda: self.elevator.below(), self.elevator))

        ## Intake
        NamedCommands.registerCommand("intakeConsume", InstantCommand(lambda: self.intake.consume(), self.intake))
        NamedCommands.registerCommand("intakeDisencumber", InstantCommand(lambda: self.intake.disencumber(), self.intake))
        NamedCommands.registerCommand("intakeStop", InstantCommand(lambda: self.intake.hold(), self.intake))
        
        ## Pivot
        NamedCommands.registerCommand("pivotAmp", InstantCommand(lambda: self.intake.pivotAmp(), self.intake))
        NamedCommands.registerCommand("pivotStow", InstantCommand(lambda: self.intake.pivotStow(), self.intake))
        NamedCommands.registerCommand("pivotGrab", InstantCommand(lambda: self.intake.pivotDown(), self.intake))
        
        """Sendables!!!"""
        self.start_chooser = SendableChooser()
        self.start_chooser.setDefaultOption("(0, 0)", Pose2d())
        self.start_chooser.addOption("Blue Amp", Pose2d(1.41, 7.29, Rotation2d()))
        self.start_chooser.addOption("Blue Speaker", Pose2d(1.34, 5.48, Rotation2d()))
        self.start_chooser.addOption("Blue Source", Pose2d(0.52, 2.1, Rotation2d()))
        self.start_chooser.addOption("Red Amp", Pose2d(15.132, 7.29, Rotation2d.fromDegrees(180)))
        self.start_chooser.addOption("Red Speaker", Pose2d(15.202, 5.48, Rotation2d.fromDegrees(180)))
        self.start_chooser.addOption("Red Source", Pose2d(16.022, 2.1, Rotation2d.fromDegrees(180)))
        
        self.start_chooser.onChange(lambda pose: self.swerve.reset_odometry(pose=pose))
        SmartDashboard.putData("Starting Position", self.start_chooser)
        
        self.auto_chooser = SendableChooser()
        self.auto_chooser.setDefaultOption("2.5 Amp", PathPlannerAuto("2.5NoteAmp"))
        self.auto_chooser.addOption("2 Amp", PathPlannerAuto("2NoteAmp"))
        self.auto_chooser.addOption("1 Amp", PathPlannerAuto("1NoteAmp"))
        self.auto_chooser.addOption("0 Amp", PathPlannerAuto("0NoteAmp"))
        self.auto_chooser.addOption("0 Speaker", PathPlannerAuto("0NoteSpeaker"))
        self.auto_chooser.addOption("0 Source", PathPlannerAuto("0NoteSource"))
        self.auto_chooser.addOption("1 Source", PathPlannerAuto("1NoteSource"))
        self.auto_chooser.addOption("1 Source Disrupt", PathPlannerAuto("1DisruptSource"))
        self.auto_chooser.addOption("1 Source Disrupt to Ready", PathPlannerAuto("1DisruptSourceToReady"))
        self.auto_chooser.addOption("1 Source Disrupt to Ready (Long)", PathPlannerAuto("1DisruptSourceToReadyLong"))
        SmartDashboard.putData("Autonomous Select", self.auto_chooser)

        self.driverController = XboxController(ExternalConstants.DRIVERCONTROLLER)
        self.functionsController = XboxController(ExternalConstants.FUNCTIONSCONTROLLER) 
        
        self.swerve.setDefaultCommand(DriveByController(self.swerve, self.driverController))

        JoystickButton(self.functionsController, XboxController.Button.kLeftBumper).onTrue(IntakeAndStow(self.intake).andThen(VibrateController(self.driverController, XboxController.RumbleType.kBothRumble, 0.75)))
        JoystickButton(self.functionsController, XboxController.Button.kRightBumper).onTrue(self.intake.runOnce(self.intake.disencumber)).onFalse(self.intake.runOnce(self.intake.hold))
        
        JoystickButton(self.functionsController, XboxController.Button.kA).onTrue(self.elevator.runOnce(self.elevator.below).alongWith(self.intake.runOnce(self.intake.pivotStow)))

        JoystickButton(self.functionsController, XboxController.Button.kB).whileTrue(ManualElevator(self.functionsController, self.elevator))
        JoystickButton(self.functionsController, XboxController.Button.kX).onTrue(self.intake.runOnce(self.intake.pivotStow).andThen(self.intake.runOnce(self.intake.hold)))
        JoystickButton(self.functionsController, XboxController.Button.kY).onTrue(self.elevator.runOnce(self.elevator.up).alongWith(self.intake.runOnce(self.intake.pivotAmp)))
        
        JoystickButton(self.driverController, XboxController.Button.kB).onTrue(ResetPivot(self.intake))
        JoystickButton(self.driverController, XboxController.Button.kX).onTrue(self.intake.runOnce(self.intake.pivotStow).andThen(self.intake.runOnce(self.intake.hold)))
        
    def getAuto(self) -> PathPlannerAuto:
        return self.auto_chooser.getSelected()
        
    def runSelectedAutoCommand(self) -> None:
        self.swerve.reset_yaw().reset_odometry(self.start_chooser.getSelected())
        self.getAuto().schedule()
