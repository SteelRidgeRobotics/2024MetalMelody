import wpilib
from commands2.button import JoystickButton
from constants import *
# from subsystems.drive import SwerveDrive
from subsystems.intake import IntakeAndPivot
from subsystems.elevator import Elevator
from commands.intake_commands import *
from commands2 import InstantCommand

class RobotContainer:
    
    def __init__(self):

        self.driverController = wpilib.XboxController(ExternalConstants.DRIVERCONTROLLER)
        self.functionsController = wpilib.XboxController(ExternalConstants.FUNCTIONSCONTROLLER)

        # self.swerve = SwerveDrive()
        self.intake = IntakeAndPivot()
        self.elevator = Elevator()

        JoystickButton(self.functionsController, wpilib.XboxController.Button.kA).whileTrue(FeederTest(self.intake))
        JoystickButton(self.functionsController, wpilib.XboxController.Button.kY).whileTrue(FeederTestDrop(self.intake))
        JoystickButton(self.functionsController, wpilib.XboxController.Button.kB).whileTrue(FeederTestStop(self.intake))
        JoystickButton(self.functionsController, wpilib.XboxController.Button.kX).onTrue(InstantCommand(lambda: self.elevator.togglePosition()))
        JoystickButton(self.functionsController, wpilib.XboxController.Button.kB).onTrue(InstantCommand(lambda: MovePivot(self.intake)))