import commands2
import constants
from subsystems.intake import Intake

class DriveControllerDefault(commands2.CommandBase):
    def __init__(self, Intake: Intake,

    def initialize(self) -> None:
        super().__init__()
        self.intake = Intake
        self.x = x
        self.y = y
        self.rightx = rightx
        self.leftTrigger = leftTrigger
        self.rightTrigger = rightTrigger

    def execute(self) -> None:
        
        if self.leftTrigger():
            self.intake.consume(self)

        if self.rightTrigger():
            self.intake.disencumber(self)