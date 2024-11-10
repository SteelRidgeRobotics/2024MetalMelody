from commands2 import TimedCommandRobot
from phoenix6.signal_logger import SignalLogger
from wpilib import Color, TimedRobot
from robotcontainer import RobotContainer
from wpilib import DriverStation, RobotBase
from wpilib.cameraserver import CameraServer
from subsystems.leds import *
from subsystems.leds.patterns import *


class MetalMelody(TimedCommandRobot):

    def __init__(self, period: float = TimedRobot.kDefaultPeriod / 1000) -> None:
        super().__init__(period)

    def robotInit(self):
        DriverStation.silenceJoystickConnectionWarning(True)
        
        SignalLogger.enable_auto_logging(False)
        
        self.container = RobotContainer()
        
        if RobotBase.isReal():
            CameraServer.launch()
            
    def robotPeriodic(self) -> None:
        self.container.updateMatchTime()
    
    def _simulationPeriodic(self) -> None:
        pass
        
    def disabledInit(self) -> None:
        pass
        
    def disabledPeriodic(self) -> None:
        pass
        
    def autonomousInit(self) -> None:
        self.container.led.set_pattern(Zone.MAIN, SimpleLedPattern.solid(Color(0,0,255)), PatternLevel.DEFAULT)

        self.container.drivetrain.reset_yaw()
        self.container.runSelectedAutoCommand()
    
    def autonomousPeriodic(self) -> None:
        pass

    def teleopInit(self) -> None:
        self.container.led.set_pattern(Zone.MAIN, LedPatternRainbow(2), PatternLevel.DEFAULT)

    def teleopPeriodic(self) -> None:
        pass

    def testInit(self) -> None:
        self.container.led.set_pattern(Zone.MAIN, SimpleLedPattern.solid(Color.kRed), PatternLevel.DEFAULT)

    def testExit(self) -> None:
        SignalLogger.stop()
