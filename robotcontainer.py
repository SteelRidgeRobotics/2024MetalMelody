from commands.drive import DriveByController
from commands.intake_and_stow import IntakeAndStow
from commands.rev_launcher import RevLauncher
from commands.manual_lift import ManualLift
from commands.vibrate import VibrateController
from commands2.button import JoystickButton, Trigger
from constants import *
from pathplannerlib.auto import NamedCommands, PathPlannerAuto
from phoenix6.controls import DutyCycleOut
from subsystems.lift import Lift
from subsystems.intake import Intake
from subsystems.pivot import Pivot, PivotStates
from subsystems.swivel import Swivel
from subsystems.swerve import Swerve
from subsystems.launcher import Launcher
from subsystems.indexer import Indexer
from wpilib import SendableChooser, SmartDashboard, Timer, XboxController
from wpimath.geometry import Pose2d, Rotation2d
from frc6343.controller.deadband import deadband

class RobotContainer:
    
    def __init__(self):
        self.swerve: Swerve = Swerve()
        self.swivel: Swivel = Swivel()
        self.lift: Lift = Lift()
        self.intake: Intake = Intake()
        self.pivot: Pivot = Pivot()
        self.launcher: Launcher = Launcher()
        self.indexer: Indexer = Indexer()
        self.swerve.initialize()
                
        # PathPlanner Commands        
        ## Lift
        
        NamedCommands.registerCommand("liftExtend", self.lift.runOnce(self.lift.raiseFull))
        NamedCommands.registerCommand("liftScore", self.lift.runOnce(self.lift.scoreShoot))
        NamedCommands.registerCommand("liftCompress", self.lift.runOnce(self.lift.compressFull))

        ## Intake
        NamedCommands.registerCommand("intakeConsume", self.intake.runOnce(self.intake.consume))
        NamedCommands.registerCommand("intakeDisencumber", self.intake.runOnce(self.intake.disencumber))
        NamedCommands.registerCommand("intakeStop", self.intake.runOnce(self.intake.stop))
        
        ## Pivot
        NamedCommands.registerCommand("pivotScore", self.pivot.runOnce(self.pivot.scoreUpwards))
        NamedCommands.registerCommand("pivotDrop", self.pivot.runOnce(self.pivot.scoreDownwards))
        NamedCommands.registerCommand("pivotStow", self.pivot.runOnce(self.pivot.stow))
        NamedCommands.registerCommand("pivotIntake", self.pivot.runOnce(self.pivot.intake))
        
        ## Misc
        NamedCommands.registerCommand("shootUpwards", self.pivot.runOnce(lambda: self.pivot.pivotMotor.set_control(DutyCycleOut(0.15))).alongWith(self.intake.runOnce(self.intake.disencumber)))
        
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
        #self.auto_chooser.setDefaultOption("2.5 Amp to Ready", PathPlannerAuto("2.5NoteToReady"))
        #self.auto_chooser.setDefaultOption("2.5 Amp", PathPlannerAuto("2.5NoteAmp"))
        self.auto_chooser.setDefaultOption("2 Amp", PathPlannerAuto("2NoteAmp"))
        self.auto_chooser.addOption("1 Amp", PathPlannerAuto("1NoteAmp"))
        self.auto_chooser.addOption("0 Amp", PathPlannerAuto("0NoteAmp"))
        self.auto_chooser.addOption("0 Speaker", PathPlannerAuto("0NoteSpeaker"))
        self.auto_chooser.addOption("0 Source", PathPlannerAuto("0NoteSource"))
        self.auto_chooser.addOption("5 Disrupt Source", PathPlannerAuto("5DisruptSource"))
        self.auto_chooser.addOption("5 Disrupt Amp", PathPlannerAuto("5DisruptAmp"))
        #self.auto_chooser.addOption("1 Source", PathPlannerAuto("1NoteSource"))
        #self.auto_chooser.addOption("1 Source Disrupt", PathPlannerAuto("1DisruptSource"))
        self.auto_chooser.addOption("Do Nothing :(", self.pivot.runOnce(self.pivot.stow).alongWith(self.lift.runOnce(self.lift.compressFull)))
        #self.auto_chooser.addOption("1 Source Disrupt to Ready", PathPlannerAuto("1DisruptSourceToReady"))
        #self.auto_chooser.addOption("1 Source Disrupt to Ready (Long)", PathPlannerAuto("1DisruptSourceToReadyLong"))
        SmartDashboard.putData("Autonomous Select", self.auto_chooser)

        self.driverController = XboxController(ExternalConstants.DRIVERCONTROLLER)
        self.functionsController = XboxController(ExternalConstants.FUNCTIONSCONTROLLER) 
        
        self.swerve.setDefaultCommand(DriveByController(self.swerve, self.driverController))

        #intake and stow
        JoystickButton(self.functionsController, XboxController.Button.kLeftBumper).onTrue(IntakeAndStow(self.intake, self.pivot, self.indexer)
                                                                                           .onlyIf(deadband(self.functionsController.getLeftTriggerAxis(), ExternalConstants.TRIGGER_DEADBAND) == 0)
                                                                                           .andThen(VibrateController(self.driverController, XboxController.RumbleType.kBothRumble, 0.75))
                                                                                           .alongWith(VibrateController(self.functionsController, XboxController.RumbleType.kBothRumble, 0.25)))
        
        #send to indexer
        JoystickButton(self.functionsController, XboxController.Button.kLeftBumper).onTrue(IntakeAndStow(self.intake, self.pivot, self.indexer, True)
                                                                                           .onlyIf(deadband(self.functionsController.getLeftTriggerAxis(), ExternalConstants.TRIGGER_DEADBAND) != 0)
                                                                                           .andThen(VibrateController(self.driverController, XboxController.RumbleType.kBothRumble, 0.75))
                                                                                           .alongWith(VibrateController(self.functionsController, XboxController.RumbleType.kBothRumble, 0.25))
                                                                                           )

        #fire
        JoystickButton(self.functionsController, XboxController.Button.kRightBumper).onTrue(RevLauncher(self.launcher)
                                                                                      .onlyIf(deadband(self.functionsController.getLeftTriggerAxis(), ExternalConstants.TRIGGER_DEADBAND) != 0)     
                                                                                      .andThen(self.indexer.runOnce(self.indexer.swallow()).withTimeout(.75))
                                                                                      .andThen(self.indexer.runOnce(self.indexer.stop())
                                                                                      .alongWith(self.launcher.runOnce(self.launcher.stop()))))
                                                                      

        #disencumber
        JoystickButton(self.functionsController, XboxController.Button.kRightBumper).onTrue(self.pivot.runOnce(lambda: self.pivot.pivotMotor.set_control(DutyCycleOut(0.1)))
                                                                                            .onlyIf(lambda: self.pivot.getState() is PivotStates.SCORE_UP and deadband(self.functionsController.getLeftTriggerAxis(), ExternalConstants.TRIGGER_DEADBAND) == 0)
                                                                                            .alongWith(self.intake.runOnce(self.intake.disencumber))
                                                                                            ).onFalse(self.intake.runOnce(self.intake.stop).alongWith(self.pivot.runOnce(self.pivot.stow)))
        
        #down and stow
        JoystickButton(self.functionsController, XboxController.Button.kA).onTrue(self.lift.runOnce(self.lift.compressFull).alongWith(self.pivot.runOnce(self.pivot.stow)))

        #climb
        JoystickButton(self.functionsController, XboxController.Button.kB).whileTrue(ManualLift(self.functionsController, self.lift)
                                        .onlyIf(deadband(self.functionsController.getRightTriggerAxis(), ExternalConstants.TRIGGER_DEADBAND) != 0))
        
        #rev
        JoystickButton(self.functionsController, XboxController.Button.kB).whileTrue(self.launcher.runOnce(self.launcher.rev)
                                        .onlyIf(deadband(self.functionsController.getRightTriggerAxis(), ExternalConstants.TRIGGER_DEADBAND) == 0))

        #stow
        JoystickButton(self.functionsController, XboxController.Button.kX).onTrue(self.pivot.runOnce(self.pivot.stow).alongWith(self.intake.runOnce(self.intake.stop)))

        #amp scoring position
        JoystickButton(self.functionsController, XboxController.Button.kY).onTrue(self.lift.runOnce(self.lift.raiseFull).alongWith(self.pivot.runOnce(self.pivot.scoreDownwards)))

        #up slightly and upwards scoring position
        JoystickButton(self.functionsController, XboxController.Button.kRightStick).onTrue(self.lift.runOnce(self.lift.scoreShoot).alongWith(self.pivot.runOnce(self.pivot.scoreUpwards)))

        #up and stow
        JoystickButton(self.functionsController, XboxController.Button.kLeftStick).onTrue(self.lift.runOnce(self.lift.raiseFull).alongWith(self.pivot.runOnce(self.pivot.stow)))
        
        #stow (driver)
        JoystickButton(self.driverController, XboxController.Button.kX).onTrue(self.pivot.runOnce(self.pivot.stow).alongWith(self.intake.runOnce(self.intake.stop)))

    def getAuto(self) -> PathPlannerAuto:
        return self.auto_chooser.getSelected()
        
    def runSelectedAutoCommand(self) -> None:
        self.swerve.reset_yaw().reset_odometry(self.start_chooser.getSelected())
        self.getAuto().schedule()
        
    def updateMatchTime(self) -> None:
        SmartDashboard.putNumber("Time", Timer.getMatchTime())