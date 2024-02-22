from commands.drive import DriveByController
from commands.intake_and_stow import IntakeAndStow
from commands.manual_elevator import ManualElevator
from commands.score_in_amp import ScoreInAmp
from commands.vibrate import VibrateController
from commands2 import InstantCommand
from commands2.button import JoystickButton
from constants import *
from pathplannerlib.auto import NamedCommands, PathPlannerAuto
from phoenix6 import SignalLogger
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
        
        routines = ["quasistatic-forward", "quasistatic-reverse", "dynamic-forward", "dynamic-reverse", "none"]
        routine = SysIdRoutine(SysIdRoutine.Config(recordState=lambda state: SignalLogger.write_string("state", str(routines[state.value]))), SysIdRoutine.Mechanism(lambda volts: self.swerve.set_voltage(volts), lambda unused: self.swerve.log_motor_output(unused), self.swerve, "drivetrain"))
        
        # PathPlanner Commands
        ## Elevator
        NamedCommands.registerCommand("elevatorUp", InstantCommand(lambda: self.elevator.up()))
        NamedCommands.registerCommand("elevatorBelow", InstantCommand(lambda: self.elevator.below()))

        ## Intake
        NamedCommands.registerCommand("intakeConsume", InstantCommand(lambda: self.intake.consume()))
        NamedCommands.registerCommand("intakeDisencumber", InstantCommand(lambda: self.intake.disencumber()))
        NamedCommands.registerCommand("intakeStop", InstantCommand(lambda: self.intake.hold()))
        
        ## Pivot
        NamedCommands.registerCommand("pivotAmp", InstantCommand(lambda: self.intake.pivotAmp()))
        NamedCommands.registerCommand("pivotStow", InstantCommand(lambda: self.intake.pivotStow()))
        NamedCommands.registerCommand("pivotGrab", InstantCommand(lambda: self.intake.pivotDown()))
        
        ## Misc
        #NamedCommands.registerCommand("scoreInAmp", ScoreInAmp(self.camera, self.swerve))
        
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
        self.auto_chooser.setDefaultOption("2 Note Amp", PathPlannerAuto("2Amp"))
        self.auto_chooser.addOption("1 Note Source", PathPlannerAuto("1NoteSource"))
        self.auto_chooser.addOption("2 Note Speaker", PathPlannerAuto("2NoteSpeaker"))
        self.auto_chooser.addOption("Quasistatic Forward", routine.quasistatic(SysIdRoutine.Direction.kForward))
        self.auto_chooser.addOption("Quasistatic Reverse", routine.quasistatic(SysIdRoutine.Direction.kReverse))
        self.auto_chooser.addOption("Dynamic Forward", routine.dynamic(SysIdRoutine.Direction.kForward))
        self.auto_chooser.addOption("Dynamic Reverse", routine.dynamic(SysIdRoutine.Direction.kReverse))
        SmartDashboard.putData("Autonomous Select", self.auto_chooser)

        self.driverController = XboxController(ExternalConstants.DRIVERCONTROLLER)
        self.functionsController = XboxController(ExternalConstants.FUNCTIONSCONTROLLER) 
        
        self.swerve.setDefaultCommand(DriveByController(self.camera, self.swerve, self.driverController))
        #self.intake.pivotAmp()

        JoystickButton(self.functionsController, XboxController.Button.kLeftBumper).onTrue(IntakeAndStow(self.intake, self.functionsController)
                                                                                           .andThen(VibrateController(self.driverController, XboxController.RumbleType.kBothRumble, 0.75)))
        JoystickButton(self.functionsController, XboxController.Button.kRightBumper).onTrue(InstantCommand(lambda: self.intake.disencumber())).toggleOnFalse(InstantCommand(lambda: self.intake.hold()))
        JoystickButton(self.functionsController, XboxController.Button.kA).onTrue(InstantCommand(lambda: self.elevator.below())
                                                                                  ).onTrue(InstantCommand(lambda: self.intake.pivotDown()).andThen(InstantCommand(lambda: self.swerve.set_max_module_speed(SwerveConstants.k_max_module_speed)))
                                                                                           ).onTrue(InstantCommand(lambda: self.swerve.set_module_override_brake(True)))
        JoystickButton(self.functionsController, XboxController.Button.kB).onTrue(ManualElevator(self.elevator, self.functionsController))
        JoystickButton(self.functionsController, XboxController.Button.kX).onTrue(InstantCommand(lambda: self.elevator.below())
                                                                                  ).onTrue(InstantCommand(lambda: self.intake.pivotStow()).andThen(InstantCommand(lambda: self.swerve.set_max_module_speed(SwerveConstants.k_max_module_speed)))
                                                                                            ).onTrue(InstantCommand(lambda: self.swerve.set_module_override_brake(True)))
        JoystickButton(self.functionsController, XboxController.Button.kY).onTrue(InstantCommand(lambda: self.elevator.up())
                                                                                  ).onTrue(InstantCommand(lambda: self.intake.pivotAmp()).andThen(InstantCommand(lambda: self.swerve.set_max_module_speed(SwerveConstants.k_max_module_speed / 4)))
                                                                                            ).onTrue(InstantCommand(lambda: self.swerve.set_module_override_brake(False)))
        
    def getAuto(self) -> PathPlannerAuto:
        return self.auto_chooser.getSelected()
        
    def runSelectedAutoCommand(self) -> None:
        self.swerve.reset_yaw().reset_odometry(self.start_chooser.getSelected())
        self.getAuto().schedule()

    def updateOdometry(self) -> None:
        if self.camera.getTagId() != -1:
            self.swerve.addVisionMeasurement(self.camera.getField2dPose(), Timer.getFPGATimestamp() + self.camera.getTotalLatency())
