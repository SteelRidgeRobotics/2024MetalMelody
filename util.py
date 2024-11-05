from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.configs.cancoder_configs import CANcoderConfiguration
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.base_status_signal import StatusCode
from constants import Constants
import math

def rot_to_meters(rotations: float) -> float:
    """Converts rotations into meters."""
    
    wheel_circumference = math.pi * Constants.DrivetrainConstants.k_wheel_diameter
    return rotations * wheel_circumference

def meters_to_rots(meters: float) -> float:
    """Converts meters into rotations."""
    
    wheel_circumference = math.pi * Constants.DrivetrainConstants.k_wheel_diameter
    return meters / wheel_circumference

def rots_to_degs(rotation: float) -> float:
    """Converts rotations to degrees."""
    
    return rotation * 360

def degs_to_rots(degrees: float) -> float:
    """Converts degrees to rotations."""
    
    return degrees / 360

def degs_to_rads(degrees: float) -> float:
    """Converts degrees to radians."""

    return degrees * (math.pi/180)

def rads_to_degs(radians: float) -> float:
    """Converts radians to degrees."""
    return radians * (180/math.pi)

def rads_to_rots(radians: float) -> float:
    return radians / (2 * math.pi)

def clamp(value: float, min_value: float, max_value: float) -> float:
    """Clamps value to be between the minimum and maximum value."""
    return max(min_value, min(value, max_value))

def ensureTalonFXConfigSuccess(talon: TalonFX, config: TalonFXConfiguration, canbus: str='') -> TalonFX:
    """Attempts to apply the given config to the TalonFX. If failed, 
    it creates a new object and reattempts. Else, returns the 
    working object.

    Args:
        talon (TalonFX): The TalonFX we want to ensure correctly 
        initializes.
        config (TalonFXConfiguration): The configuration to apply to 
        the TalonFX.
        canbus (str, optional): The CAN bus the TalonFX is connected 
        to. Defaults to ''.

    Returns:
        TalonFX: The TalonFX object that did not return -1001 when 
        applying the given config.
    """

    while True:
        if talon.configurator.apply(config) is StatusCode.TX_FAILED:
            talon = TalonFX(talon.device_id, canbus=canbus)
        else:
            return talon
        
def ensureCANcoderConfigSuccess(cancoder: CANcoder, config: CANcoderConfiguration, canbus: str='') -> CANcoder:
    """Attempts to apply the given config to the CANcoder. If failed, 
    it creates a new object and reattempts. Else, returns the 
    working object.

    Args:
        cancoder (CANcoder): The CANcoder we want to ensure correctly 
        initializes.
        config (CANcoderConfiguration): The configuration to apply to 
        the CANcoder.
        canbus (str, optional): The CAN bus the CANcoder is connected 
        to. Defaults to ''.

    Returns:
        CANcoder: The CANcoder object that did not return -1001 when 
        applying the given config.
    """

    while True:
        if cancoder.configurator.apply(config) is StatusCode.TX_FAILED:
            cancoder = CANcoder(cancoder.device_id, canbus=canbus)
        else:
            return cancoder
