import wpilib
from commands2.button import JoystickButton
from constants import *
# from subsystems.drive import SwerveDrive
from subsystems.intake import IntakeAndPivot
from subsystems.elevator import Elevator
from commands.intake_commands import FeederTest
from commands.toggle_elevator import ToggleElevator

class RobotContainer:
    
    def __init__(self):

        self.driverController = wpilib.XboxController(ExternalConstants.DRIVERCONTROLLER)
        self.functionsController = wpilib.XboxController(ExternalConstants.FUNCTIONSCONTROLLER)

        # self.swerve = SwerveDrive()
        self.intake = IntakeAndPivot()
        self.elevator = Elevator()

        JoystickButton(self.functionsController, wpilib.XboxController.Button.kA).whileTrue(FeederTest())