from commands.drive import DriveByController
from commands.intake_and_stow import IntakeAndStow
from commands.manual_elevator import ManualElevator
from commands.reset_pivot import ResetPivot
from commands.vibrate import VibrateController
from commands2 import InstantCommand
from commands2.button import JoystickButton
from constants import *
from pathplannerlib.auto import NamedCommands, PathPlannerAuto
from phoenix6 import SignalLogger
from subsystems.elevator import Elevator
from subsystems.intake import Intake, IntakeStates
from subsystems.swerve import Swerve
from wpilib import SendableChooser, SmartDashboard, XboxController
from commands2.sysid import SysIdRoutine
from wpimath.geometry import Pose2d, Rotation2d

class RobotContainer:
    
    def __init__(self):
        self.swerve: Swerve = Swerve()
        self.elevator: Elevator = Elevator()
        self.intake = Intake()
        self.swerve.initialize()
        
        routines = ["quasistatic-forward", "quasistatic-reverse", "dynamic-forward", "dynamic-reverse", "none"]
        routine = SysIdRoutine(SysIdRoutine.Config(recordState=lambda state: SignalLogger.write_string("state", str(routines[state.value]))), SysIdRoutine.Mechanism(lambda volts: self.swerve.set_voltage(volts), lambda unused: self.swerve.log_motor_output(unused), self.swerve, "drivetrain"))
        
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
        self.auto_chooser.setDefaultOption("2 Note Amp", PathPlannerAuto("2NoteAmp")) 
        self.auto_chooser.addOption("0 Note Speaker", PathPlannerAuto("0NoteSpeaker"))
        self.auto_chooser.addOption("Quasistatic Forward", routine.quasistatic(SysIdRoutine.Direction.kForward))
        self.auto_chooser.addOption("Quasistatic Reverse", routine.quasistatic(SysIdRoutine.Direction.kReverse))
        self.auto_chooser.addOption("Dynamic Forward", routine.dynamic(SysIdRoutine.Direction.kForward))
        self.auto_chooser.addOption("Dynamic Reverse", routine.dynamic(SysIdRoutine.Direction.kReverse))
        self.auto_chooser.addOption("0 Note Source", PathPlannerAuto("0NoteSource"))
        self.auto_chooser.addOption("Rotation Test", PathPlannerAuto("RotationTest"))
        self.auto_chooser.addOption("Translation Test", PathPlannerAuto("TranslateTest"))
        SmartDashboard.putData("Autonomous Select", self.auto_chooser)

        self.driverController = XboxController(ExternalConstants.DRIVERCONTROLLER)
        self.functionsController = XboxController(ExternalConstants.FUNCTIONSCONTROLLER) 
        
        self.swerve.setDefaultCommand(DriveByController(self.swerve, self.driverController))

        JoystickButton(self.functionsController, XboxController.Button.kLeftBumper).onTrue(IntakeAndStow(self.intake, self.functionsController).onlyIf(lambda: self.intake.getIntakeState() is not IntakeStates.GRAB)
                                                                                           .andThen(VibrateController(self.driverController, XboxController.RumbleType.kBothRumble, 0.75)))
        JoystickButton(self.functionsController, XboxController.Button.kRightBumper).onTrue(InstantCommand(lambda: self.intake.disencumber(), self.intake).onlyIf(lambda: self.intake.getIntakeState() is not IntakeStates.TOSS)
                                                                                            ).onFalse(InstantCommand(lambda: self.intake.hold(), self.intake))
        
        JoystickButton(self.functionsController, XboxController.Button.kA).onTrue(InstantCommand(lambda: self.elevator.below(), self.elevator)
                                                                                  ).onTrue(InstantCommand(lambda: self.intake.pivotDown(), self.intake).andThen(InstantCommand(lambda: self.swerve.set_max_module_speed(SwerveConstants.k_max_module_speed)))
                                                                                           ).onTrue(InstantCommand(lambda: self.swerve.set_module_override_brake(True)))
        JoystickButton(self.functionsController, XboxController.Button.kB).whileTrue(ManualElevator(self.functionsController, self.elevator))
        JoystickButton(self.functionsController, XboxController.Button.kX).onTrue(InstantCommand(lambda: self.intake.pivotStow(), self.intake).andThen(InstantCommand(lambda: self.swerve.set_max_module_speed(SwerveConstants.k_max_module_speed)))
                                                                                  ).onTrue(InstantCommand(lambda: self.swerve.set_module_override_brake(True)))
        JoystickButton(self.functionsController, XboxController.Button.kY).onTrue(InstantCommand(lambda: self.elevator.up(), self.elevator)
                                                                                  ).onTrue(InstantCommand(lambda: self.intake.pivotAmp(), self.intake).andThen(InstantCommand(lambda: self.swerve.set_max_module_speed(SwerveConstants.k_max_module_speed / 4)))
                                                                                            ).onTrue(InstantCommand(lambda: self.swerve.set_module_override_brake(False)))
        
        JoystickButton(self.driverController, XboxController.Button.kB).onTrue(ResetPivot(self.intake))
        
    def getAuto(self) -> PathPlannerAuto:
        return self.auto_chooser.getSelected()
        
    def runSelectedAutoCommand(self) -> None:
        self.swerve.reset_yaw().reset_odometry(self.start_chooser.getSelected())
        self.getAuto().schedule()
