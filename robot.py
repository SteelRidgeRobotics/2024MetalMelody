import commands2
from commands2.timedcommandrobot import seconds
from wpilib import TimedRobot
from robotcontainer import RobotContainer

class DumpsterFire(commands2.TimedCommandRobot):

    def __init__(self, period: float = TimedRobot.kDefaultPeriod / 1000) -> None:
        super().__init__(period)

    def robotInit(self):

        self.container = RobotContainer()

    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()
        
    def autonomousInit(self) -> None:
        self.container.runSelectedAutoCommand()
        