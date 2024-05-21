from commands2 import Command
from constants import *
from wpilib import SmartDashboard
from commands.mode_toggle import ModeToggle as mode_toggle
from subsystems.intake import Intake
from subsystems.launcher import Launcher
from subsystems.indexer import Indexer


class Fire(Command):
    def __init__(self):
        super().__init__()

    def execute(self):
        if mode_toggle.get_mode() == Modes.INTAKE:
            Intake.disencumber()
        else:
            Indexer.launcherWise()
            Launcher.launch()

        SmartDashboard.putBoolean("Working????? ", True)

    def isFinished(self) -> bool:

        return super().isFinished()
    
    def end(self, interrupted: bool):
        Indexer.stop()
        Intake.stop()
        return super().end(interrupted)

        