import wpilib
from commands2.button import JoystickButton
from constants import *
# from subsystems.drive import SwerveDrive
from subsystems.intake import IntakeAndPivot
from subsystems.elevator import Elevator
from commands.intake_commands import *
# from commands.toggle_elevator import ToggleElevator
from commands.move_elevator import MoveElevator
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

        #self.elevator.setDefaultCommand(MoveElevator(self.elevator))

        

        