from commands2 import Command
from subsystems.intake import Intake
from subsystems.pivot import Pivot
from subsystems.indexer import Indexer

class IntakeAndStow(Command):
    
    def __init__(self, intake: Intake, pivot: Pivot, indexer: Indexer, ignore=False):
        super().__init__()
        
        self.intake = intake
        self.pivot = pivot
        self.ignore = ignore
        self.indexer = indexer
        self.addRequirements(self.intake, self.pivot)
        
    def initialize(self):
        self.pivot.intake()
        self.intake.consume()
        
    def isFinished(self) -> bool:
        if not self.ignore:
            return self.intake.beam_breaker.get()
        else:
            return self.indexer.beam_breaker.get()
    
    def end(self, interrupted: bool):
        self.intake.stop()
        self.indexer.stop()
        if not interrupted:
            self.pivot.stow()