from commands.drive import DriveByController
from commands2 import InstantCommand, ParallelCommandGroup
from commands2.button import JoystickButton
from constants import *
from pathplannerlib.auto import NamedCommands, PathPlannerAuto
from subsystems.camera import Camera
from subsystems.elevator import Elevator
from subsystems.intake import Intake
from subsystems.swerve import Swerve
from wpilib import SendableChooser, SmartDashboard, Timer, XboxController
from commands2.sysid import SysIdRoutine
from wpimath.geometry import Pose2d, Rotation2d

class RobotContainer:
    
    def __init__(self):
        self.camera: Camera = Camera()
        self.elevator: Elevator = Elevator()
        self.intake = Intake()
        self.swerve: Swerve = Swerve()
        self.swerve.initialize()
        
        routine = SysIdRoutine(SysIdRoutine.Config(), SysIdRoutine.Mechanism(lambda volts: self.swerve.set_voltage(volts), lambda unused: self.swerve.log_motor_output(unused), self.swerve, "drivetrain"))
        
        # PathPlanner Commands
        ## Elevator
        NamedCommands.registerCommand("elevatorUp", InstantCommand(lambda: self.elevator.setStage(0)))
        NamedCommands.registerCommand("elevatorMid", InstantCommand(lambda: self.elevator.setStage(1)))
        NamedCommands.registerCommand("elevatorBottom", InstantCommand(lambda: self.elevator.setStage(2)))

        ## Intake
        NamedCommands.registerCommand("intakeConsume", InstantCommand(lambda: self.intake.consume()))
        NamedCommands.registerCommand("intakeDisencumber", InstantCommand(lambda: self.intake.disencumber()))
        NamedCommands.registerCommand("intakeStop", InstantCommand(lambda: self.intake.hold()))
        
        ## Pivot
        NamedCommands.registerCommand("pivotAmp", InstantCommand(lambda: self.intake.pivotAmp()))
        NamedCommands.registerCommand("pivotStow", InstantCommand(lambda: self.intake.pivotStow()))
        NamedCommands.registerCommand("pivotGrab", InstantCommand(lambda: self.intake.pivotDown()))
        
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
        self.auto_chooser.addOption("Quasistatic Forward", routine.quasistatic(routine.Direction.kForward))
        self.auto_chooser.addOption("Quasistatic Reverse", routine.quasistatic(routine.Direction.kReverse))
        self.auto_chooser.addOption("Dynamic Forward", routine.dynamic(routine.Direction.kForward))
        self.auto_chooser.addOption("Dynamic Reverse", routine.dynamic(SysIdRoutine.Direction.kReverse))
        SmartDashboard.putData("Autonomous Select", self.auto_chooser)

        self.driverController = XboxController(ExternalConstants.DRIVERCONTROLLER)
        self.functionsController = XboxController(ExternalConstants.FUNCTIONSCONTROLLER) 
        
        self.swerve.setDefaultCommand(DriveByController(self.camera, self.swerve, self.driverController))

        JoystickButton(self.functionsController, XboxController.Button.kLeftBumper).toggleOnTrue(InstantCommand(lambda: self.intake.consume())).toggleOnFalse(InstantCommand(lambda: self.intake.hold()))
        JoystickButton(self.functionsController, XboxController.Button.kRightBumper).onTrue(InstantCommand(lambda: self.intake.disencumber())).toggleOnFalse(InstantCommand(lambda: self.intake.hold()))
        JoystickButton(self.functionsController, XboxController.Button.kX).onTrue(InstantCommand(lambda: self.intake.pivotDown()))
        JoystickButton(self.functionsController, XboxController.Button.kB).onTrue(InstantCommand(lambda: self.intake.pivotStow()))
        # JoystickButton(self.functionsController, XboxController.Button.kB).onTrue(InstantCommand(lambda: self.intake.pivotCycle()))
        # JoystickButton(self.functionsController, XboxController.Button.kX).onTrue(InstantCommand(lambda: self.elevator.togglePosition()))
        ###JoystickButton(self.functionsController, XboxController.Button.kA).onTrue(ParallelCommandGroup(InstantCommand(lambda: self.elevator.below()), InstantCommand(lambda: self.intake.pivotDown())))
        ###JoystickButton(self.functionsController, XboxController.Button.kB).onTrue(ParallelCommandGroup(InstantCommand(lambda: self.elevator.below()), InstantCommand(lambda: self.intake.pivotStow())))
        ###JoystickButton(self.functionsController, XboxController.Button.kY).onTrue(ParallelCommandGroup(InstantCommand(lambda: self.elevator.up()), InstantCommand(lambda: self.intake.pivotAmp())))
        JoystickButton(self.functionsController, XboxController.Button.kY).onTrue(InstantCommand(lambda: self.elevator.up()))
        # JoystickButton(self.functionsController, XboxController.Button.kB).onTrue(InstantCommand(lambda: self.intake.pivotStow()))
        JoystickButton(self.functionsController, XboxController.Button.kA).onTrue(InstantCommand(lambda: self.elevator.below()))
        
    def getAuto(self) -> PathPlannerAuto:
        return self.auto_chooser.getSelected()
        
    def runSelectedAutoCommand(self) -> None:
        self.swerve.reset_yaw().reset_odometry(self.start_chooser.getSelected())
        self.getAuto().schedule()

    def updateOdometry(self) -> None:
        if self.camera.getTagId() != -1:
            self.swerve.addVisionMeasurement(self.camera.getField2dPose(), Timer.getFPGATimestamp() + self.camera.getTotalLatency())
