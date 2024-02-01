import wpilib
import commands2
from subsystems.intake import IntakeAndPivot
from commands2.CommandXboxController

class Consume(commands2.CommandBase):

    def __init__(self, IntakeAndPivot):
        self.intake = IntakeAndPivot
        self.addRequirements(self.intake)

    def execute(self):
        
        self.intake.consume()

