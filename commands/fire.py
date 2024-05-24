from commands2 import Command
from constants import *
from wpilib import SmartDashboard
from commands.mode_toggle import ModeToggle as mode_toggle
from subsystems.intake import Intake
from subsystems.launcher import Launcher
from subsystems.indexer import Indexer


class Fire(Command):
    def __init__(self, intake : Intake, indexer : Indexer, launcher : Launcher):
        super().__init__()
        self.intake = intake
        self.indexer = indexer
        self.launcher = launcher


    def execute(self):

        if mode_toggle.get_mode() == Modes.INTAKE:
            self.intake.disencumber()
            SmartDashboard.putBoolean("Disencumbering", True)
        else:
            self.indexer.launcherWise()
            self.launcher.launch()
        
        SmartDashboard.putBoolean("Launching", True)

    def isFinished(self) -> bool:
        
        return True
    
    def end(self, interrupted: bool):
        
        SmartDashboard.putBoolean("Disencumbering", False)
        SmartDashboard.putBoolean("Launching", False)
        
        self.indexer.stop()
        self.intake.stop()
        self.launcher.stop()
        return super().end(interrupted)

        