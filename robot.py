import commands2
from commands2.timedcommandrobot import seconds
from wpilib import TimedRobot
from robotcontainer import RobotContainer
from wpilib.cameraserver import CameraServer

class DumpsterFire(commands2.TimedCommandRobot):

    def __init__(self, period: float = TimedRobot.kDefaultPeriod / 1000) -> None:
        super().__init__(period)

    def robotInit(self):
        CameraServer.launch('vision.py')
        self.container = RobotContainer()

    def robotPeriodic(self):
        self.container.updateOdometry()
        commands2.CommandScheduler.getInstance().run()
        
    def autonomousInit(self) -> None:
        self.container.runSelectedAutoCommand()
        