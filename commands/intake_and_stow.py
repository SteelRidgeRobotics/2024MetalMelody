from commands2 import Command
from subsystems.intake import Intake
from subsystems.pivot import Pivot

class IntakeAndStow(Command):
    
    def __init__(self, intake: Intake, pivot: Pivot):
        super().__init__()
        
        self.intake = intake
        self.pivot = pivot
        self.addRequirements(self.intake, self.pivot)
        
    def initialize(self):
        self.pivot.intake()
        self.intake.consume()
        
    def isFinished(self) -> bool:
        return self.intake.hasNote()
    
    def end(self, interrupted: bool):
        if not interrupted:
            self.pivot.stow()