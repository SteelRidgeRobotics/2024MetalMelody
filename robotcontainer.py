import wpilib
from commands2.button import JoystickButton
from constants import *
from subsystems.camera import Camera
from subsystems.intake import IntakeAndPivot
from subsystems.elevator import Elevator
from subsystems.swerve import Swerve
from commands.intake_commands import *
from commands.drive import DriveByController
from commands2 import InstantCommand
from pathplannerlib.auto import PathPlannerAuto
from wpilib import SendableChooser, SmartDashboard, Timer
from wpimath.geometry import Pose2d, Rotation2d

class RobotContainer:
    
    def __init__(self):
        self.camera: Camera = Camera()
        self.swerve: Swerve = Swerve() # This helps IntelliSense know that this is a Swerve object, not a Subsystem (it gets confused sometimes)
        self.intake = IntakeAndPivot()
        self.elevator = Elevator()
        
        """Sendables!!!"""
        self.start_chooser = SendableChooser()
        self.start_chooser.setDefaultOption("(0, 0)", Pose2d())
        self.start_chooser.addOption("Blue Amp", Pose2d(0.48, 7.32, Rotation2d()))
        self.start_chooser.addOption("Blue Speaker", Pose2d(1.34, 5.48, Rotation2d()))
        self.start_chooser.addOption("Blue Source", Pose2d(0.52, 2.1, Rotation2d()))
        self.start_chooser.addOption("Red Amp", Pose2d(16.062, 7.32, Rotation2d()))
        self.start_chooser.addOption("Red Speaker", Pose2d(15.202, 5.48, Rotation2d()))
        self.start_chooser.addOption("Red Source", Pose2d(16.022, 2.1, Rotation2d()))
        
        self.start_chooser.onChange(lambda pose: self.swerve.reset_odometry(pose=pose))
        SmartDashboard.putData("Starting Position", self.start_chooser)
        
        self.auto_chooser = SendableChooser()
        self.auto_chooser.setDefaultOption("2 Note Amp", PathPlannerAuto("2NoteAmp"))
        self.auto_chooser.addOption("1 Note Source", PathPlannerAuto("1NoteSource"))
        self.auto_chooser.addOption("2 Note Speaker", PathPlannerAuto("2NoteSpeaker"))
        SmartDashboard.putData("Autonomous Select", self.auto_chooser)

        self.driverController = wpilib.XboxController(ExternalConstants.DRIVERCONTROLLER)
        self.functionsController = wpilib.XboxController(ExternalConstants.FUNCTIONSCONTROLLER) 

        self.robot_pose = self.getStartPose()
        
        self.swerve.setDefaultCommand(DriveByController(self.swerve, self.driverController))

        JoystickButton(self.functionsController, wpilib.XboxController.Button.kA).whileTrue(FeederTest(self.intake))
        JoystickButton(self.functionsController, wpilib.XboxController.Button.kY).whileTrue(FeederTestDrop(self.intake))
        JoystickButton(self.functionsController, wpilib.XboxController.Button.kB).whileTrue(FeederTestStop(self.intake))
        JoystickButton(self.functionsController, wpilib.XboxController.Button.kX).onTrue(InstantCommand(lambda: self.elevator.togglePosition()))
        JoystickButton(self.functionsController, wpilib.XboxController.Button.kB).onTrue(InstantCommand(lambda: MovePivot(self.intake)))
        
    def getAuto(self) -> PathPlannerAuto:
        return self.auto_chooser.getSelected()
    
    def getStartPose(self) -> Pose2d:
        return self.start_chooser.getSelected()
        
    def runSelectedAutoCommand(self) -> None:
        self.swerve.reset_yaw().reset_odometry(self.start_chooser.getSelected())
        self.getAuto().schedule()

    def updateOdometry(self) -> None:
        self.swerve.addVisionMeasurement(self.camera.getField2dPose(), Timer.getFPGATimestamp() + self.camera.getTotalLatency())
        