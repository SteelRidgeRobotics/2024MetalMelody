from commands2 import Command
from subsystems.intake import Intake
from wpilib import XboxController

class IntakeAndStow(Command):
    
    def __init__(self, intake: Intake, functions: XboxController):
        super().__init__()
        
        self.intake = intake
        self.functions = functions
        
        self.addRequirements(self.intake)
        
    def initialize(self):
        self.intake.consume()
        
    def isFinished(self) -> bool:
        return self.intake.hasNote()
    
    def end(self, interrupted: bool):
        if not interrupted:
            self.intake.pivotStow()