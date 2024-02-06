import commands2
from robotcontainer import RobotContainer

class DumpsterFire(commands2.TimedCommandRobot):

    def __init__(self):

        super().__init__()

    def robotInit(self):

        self.container = RobotContainer()

    def robotPeriodic(self):
        
        commands2.CommandScheduler.getInstance().run()
        