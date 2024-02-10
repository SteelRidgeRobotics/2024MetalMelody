import commands2
from robotcontainer import RobotContainer
from wpilib.cameraserver import CameraServer
import cv2
import wpilib

class DumpsterFire(commands2.TimedCommandRobot):

    def __init__(self):

        super().__init__()

    def robotInit(self):
        wpilib.CameraServer.launch('vision.py:main')
        self.container = RobotContainer()

    def robotPeriodic(self):
        
        commands2.CommandScheduler.getInstance().run()
        