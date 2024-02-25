from commands.drive import DriveByController
from commands.intake_and_stow import IntakeAndStow
from commands.reset_pivot import ResetPivot
from commands.vibrate import VibrateController
from commands2 import InstantCommand
from commands2.button import JoystickButton
from commands2.button import Trigger
from constants import *
from frc6343.controller.deadband import deadband
from pathplannerlib.auto import NamedCommands, PathPlannerAuto
from phoenix6 import SignalLogger
from subsystems.camera import Camera
from subsystems.elevator import Elevator
from subsystems.intake import Intake, IntakeStates
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
        self.auto_chooser.setDefaultOption("2 Note Amp", PathPlannerAuto("2NoteAmp"))
        self.auto_chooser.addOption("0 Note Speaker", PathPlannerAuto("0NoteSpeaker"))
        self.auto_chooser.addOption("Quasistatic Forward", routine.quasistatic(SysIdRoutine.Direction.kForward))
        self.auto_chooser.addOption("Quasistatic Reverse", routine.quasistatic(SysIdRoutine.Direction.kReverse))
        self.auto_chooser.addOption("Dynamic Forward", routine.dynamic(SysIdRoutine.Direction.kForward))
        self.auto_chooser.addOption("Dynamic Reverse", routine.dynamic(SysIdRoutine.Direction.kReverse))
        self.auto_chooser.addOption("0 Note Source", PathPlannerAuto("0NoteSource"))
        SmartDashboard.putData("Autonomous Select", self.auto_chooser)

        self.driverController = XboxController(ExternalConstants.DRIVERCONTROLLER)
        self.functionsController = XboxController(ExternalConstants.FUNCTIONSCONTROLLER) 
        
        self.swerve.setDefaultCommand(DriveByController(self.camera, self.swerve, self.driverController))
        #self.intake.pivotAmp()

        JoystickButton(self.functionsController, XboxController.Button.kLeftBumper).onTrue(IntakeAndStow(self.intake, self.functionsController).onlyIf(lambda: self.intake.getIntakeState() is not IntakeStates.GRAB)
                                                                                           .andThen(VibrateController(self.driverController, XboxController.RumbleType.kBothRumble, 0.75)))
        JoystickButton(self.functionsController, XboxController.Button.kRightBumper).onTrue(InstantCommand(lambda: self.intake.disencumber(), self.intake).onlyIf(lambda: self.intake.getIntakeState() is not IntakeStates.TOSS)
                                                                                            ).onFalse(InstantCommand(lambda: self.intake.hold(), self.intake))
        
        Trigger(lambda: abs(getTriggerCombinedValue(self.functionsController.getLeftTriggerAxis(), self.functionsController.getRightTriggerAxis())) > 0
                ).whileTrue(InstantCommand(lambda: self.elevator.setDutyCycle(getTriggerCombinedValue(self.functionsController.getLeftTriggerAxis(), self.functionsController.getRightTriggerAxis()))).repeatedly())
        
        JoystickButton(self.functionsController, XboxController.Button.kA).onTrue(InstantCommand(lambda: self.elevator.below(), self.elevator)
                                                                                  ).onTrue(InstantCommand(lambda: self.intake.pivotDown(), self.intake).andThen(InstantCommand(lambda: self.swerve.set_max_module_speed(SwerveConstants.k_max_module_speed)))
                                                                                           ).onTrue(InstantCommand(lambda: self.swerve.set_module_override_brake(True)))
        JoystickButton(self.functionsController, XboxController.Button.kB).onTrue(ResetPivot(self.intake))
        JoystickButton(self.functionsController, XboxController.Button.kX).onTrue(InstantCommand(lambda: self.elevator.below(), self.elevator)
                                                                                  ).onTrue(InstantCommand(lambda: self.intake.pivotStow(), self.intake).andThen(InstantCommand(lambda: self.swerve.set_max_module_speed(SwerveConstants.k_max_module_speed)))
                                                                                            ).onTrue(InstantCommand(lambda: self.swerve.set_module_override_brake(True)))
        JoystickButton(self.functionsController, XboxController.Button.kY).onTrue(InstantCommand(lambda: self.elevator.up(), self.elevator)
                                                                                  ).onTrue(InstantCommand(lambda: self.intake.pivotAmp(), self.intake).andThen(InstantCommand(lambda: self.swerve.set_max_module_speed(SwerveConstants.k_max_module_speed / 4)))
                                                                                            ).onTrue(InstantCommand(lambda: self.swerve.set_module_override_brake(False)))
        
    def getAuto(self) -> PathPlannerAuto:
        return self.auto_chooser.getSelected()
        
    def runSelectedAutoCommand(self) -> None:
        self.swerve.reset_yaw().reset_odometry(self.start_chooser.getSelected())
        self.getAuto().schedule()

    def updateOdometry(self) -> None:
        if self.camera.getTagId() != -1:
            self.swerve.addVisionMeasurement(self.camera.getField2dPose(), Timer.getFPGATimestamp() + self.camera.getTotalLatency())
            
def getTriggerCombinedValue(leftTrigger: float, rightTrigger: float) -> float:
    return deadband(rightTrigger, ExternalConstants.TRIGGER_DEADBAND) - deadband(leftTrigger, ExternalConstants.TRIGGER_DEADBAND)
