import typing
import commands2
from subsystems.elevator import Elevator


class ToggleElevator(commands2.CommandBase):

    def __init__(self, elevator: Elevator, button: typing.Callable([], float)):

        self.button = button
        self.elevator = elevator

        self.addRequirements([self.elevator])

    def execute(self):

        if self.button():
            self.elevator.toggle()

         

