from phoenix6.hardware import CANcoder, TalonFX
from phoenix6.configs.cancoder_configs import CANcoderConfiguration
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.base_status_signal import StatusCode

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
