'''
Sensor Driver for the Sensiron/Adafruit SCD30 sensor.

Includes:
    - an internal `_SCD30` class for i2c operations with no real abstraction
    - an `SCD30` class for simplified operations, intended for end users.

The `SCD30` class synchronizes using The `BusManager` by default, the `_SCD30` class does not.
'''

import time
import struct
import logging
from dataclasses import dataclass
from smbus2 import SMBus, i2c_msg

logger = logging.getLogger(__name__)

# Dataclasses
@dataclass(frozen=True)
class SCD30Reading:
    CO2: int
    relative_humidity: float
    temperature: float

# Constants
SCD30_ADDR = 0x61
SCD30_TRIGGER_CONTINUOUS_COMMAND = 0x0010
SCD30_STOP_CONTINUOUS_COMMAND = 0x0104
SCD30_SET_MEASUREMENT_INTERVAL_COMMAND = 0x4600
SCD30_GET_DATA_READY_COMMAND = 0x0202

# Helper methods
def to_uint16(value: int) -> int:
    '''
    Clamps an integer to [0, 65565] range.

    Raises an assertion error if this is impossible.
    '''
    assert isinstance(value, int) and 0 <= value <= 0xFFFF
    return value & 0xFFFF

def uint16_to_two_bytes(value: int) -> tuple[int, int]:
    '''
    Returns a tuple of two bytes from a uint16 in big endian order.

    Note: `value` is presumed to be a uint16, any bits other than the lowest 16 are ignored.
    '''
    return (
        (value >> 8) & 0xFF,
        (value) & 0xFF
    )

def scd30_data_crc8(data: bytes) -> int:
    '''
    CRC-8 implementation per sensor spec.
    Polynomial: 0x31
    Initial Value: 0xFF
    Final XOR: 0xFF
    '''
    crc = 0xFF
    for byte in data:
        crc ^= byte # crc = crc XOR byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x31
            else:
                crc <<= 1
            crc &= 0xFF # Keep to 8 bits
    return crc

def parse_float_with_crc(fields: list[int]) -> float:
    '''
    Reading the measurement from the SCD30 yields (mmsb, mlsb, crc, lmsb, llsb, crc) for each float value
    '''
    mmsb, mlsb, crc1, lmsb, llsb, crc2 = fields
    if scd30_data_crc8(bytes([mmsb, mlsb])) != crc1:
        raise ValueError(
            f"CRC mismatch in first word: {hex(mmsb)}, {hex(mlsb)}, intended crc: {hex(crc1)}, actual: {hex(scd30_data_crc8(bytes([mmsb, mlsb])))}"
        )
    if scd30_data_crc8(bytes([lmsb, llsb])) != crc2:
        raise ValueError(
            f"CRC mismatch in first word: {hex(lmsb)}, {hex(llsb)}, intended crc: {hex(crc2)}, actual: {hex(scd30_data_crc8(bytes([lmsb, llsb])))}"
        )        
    return struct.unpack(">f", bytes([mmsb, mlsb, lmsb, llsb]))[0]

class _SCD30:
    def __init__(self, bus: SMBus | str | int | None):
        if bus is None:
            # Fallback to default `/dev/i2c-1` case for raspberry pi usage
            try:
                self.bus = SMBus(1)
            except FileNotFoundError:
                 logger.error("_SCD30 attempted to fallback to /dev/i2c-1, but failed.\nVerify that there is a valid I2C bus on your machine.\nNote: Non-Unix-like systems are not supported.")
        
        elif isinstance(bus, str | int):
            try:
                self.bus = SMBus(bus)
            except FileNotFoundError:
                logger.error(f"_SCD30 could not initialize smbus using {bus}.\nVerify {bus} exists.")

        if isinstance(bus, SMBus):
            self.bus = bus
    
    def cleanup(self):
        '''
        Cleans up allocated resources.
         
        Should be run at the end of usage, but is intended for context management metods. 
        '''
        # Explicitly close and de-reference the bus.
        if isinstance(self.bus, SMBus):
            self.bus.close()
            self.bus = None

    def bus_exists(self) -> bool:
        return self.bus is not None and isinstance(self.bus, SMBus)
 
    def trigger_continuous_measurements(self, ambient_pressure: int | None):
        '''
        Triggers continuous measurement in the SCD30 sensor.

        Args:
            ambient_pressure (int | none): CO2 measurement will be compensated by this value. 
            ambient pressure should be 0 or [700, 1400] mBar. If 0 (or None), CO2 measurement will not
            be compensated using ambient pressure.
        '''
        if not self.bus_exists():
            logger.error("I2C Bus fails to exist, cannot read or write from I2C bus.")
            return

        
        if ambient_pressure is None:
            ambient_pressure = 0
        
        if not (ambient_pressure == 0 or 700 <= ambient_pressure <= 1400):
            logger.error("ambient_pressure parameter must be 0 or [700, 1400] mBar.") 

        # clamp to uint16
        ambient_pressure = to_uint16(ambient_pressure)

        # Convert ambient_pressure uint16 into two bytes
        pressure_high_byte, pressure_low_byte = uint16_to_two_bytes(ambient_pressure)

        # Convert the trigger continuous cmd into two bytes
        cmd_high_byte, cmd_low_byte = uint16_to_two_bytes(SCD30_TRIGGER_CONTINUOUS_COMMAND)

        # Create CRC-8 using the pressure bytes
        crc = (
            scd30_data_crc8(bytes([pressure_high_byte, pressure_low_byte]))
        ) & 0xFF # explicitly mask to 8 bits        

        # Write the bytes in big endian order
        self.bus.write_block_data(
            SCD30_ADDR,
            cmd_high_byte,
            [cmd_low_byte, pressure_high_byte, pressure_low_byte, crc]
        )
    
    def stop_continuous_measurements(self):
        '''
        Stops the continuous measurment of the SCD30
        '''
        if not self.bus_exists():
            logger.error("I2C Bus fails to exist, cannot read or write from I2C bus.")
            return
       
        cmd_high_byte, cmd_low_byte = uint16_to_two_bytes(SCD30_STOP_CONTINUOUS_COMMAND)

        self.bus.write_block_data(
            SCD30_ADDR,
            cmd_high_byte,
            [cmd_low_byte]
        )

    def set_measurement_interval(self, interval = 2):
        '''
        Sets the measurment interval used in continuous measurement mode.

        The default and minimum is 2 seconds, the maximum is 1800 seconds.
        ''' 
        if not self.bus_exists():
            logger.error("I2C Bus fails to exist, cannot read or write from I2C bus.")
            return
        
        interval = to_uint16(interval)
        
        if not 2 <= interval <= 1800:
            logger.error("Measurment interval must be [2, 1800] seconds.")
            return
        
        data_high_byte, data_low_byte = uint16_to_two_bytes(interval)
        cmd_high_byte, cmd_low_byte = uint16_to_two_bytes(SCD30_SET_MEASUREMENT_INTERVAL_COMMAND)
        crc = scd30_data_crc8(bytes([data_high_byte, data_low_byte]))

        self.bus.write_block_data(
            SCD30_ADDR,
            cmd_high_byte,
            [cmd_low_byte, data_high_byte, data_low_byte, crc]
        )

    def get_ready_status(self) -> bool:
        '''
        Check if a measurement can be read from the sensor's buffer.
        '''
        if not self.bus_exists():
            logger.error("I2C Bus fails to exist, cannot read or write from I2C bus.")
            return False
        
        cmd_high_byte, cmd_low_byte = uint16_to_two_bytes(SCD30_GET_DATA_READY_COMMAND)
        
        write_msg = i2c_msg.write(SCD30_ADDR, [cmd_high_byte, cmd_low_byte])

        read_msg = i2c_msg.read(SCD30_ADDR, 3)

        # Write to the SCD30 telling it to report data ready
        self.bus.i2c_rdwr(write_msg)
        # sleep 3ms (per the data sheet)
        self.bus.i2c_rdwr(read_msg)
        
        data = list(read_msg)
        data_high_byte, data_low_byte, crc_received = data[0], data[1], data[2]

        crc_calculated = scd30_data_crc8(bytes([data_high_byte, data_low_byte]))

        if crc_calculated != crc_received:
            logger.error(f"CRC mismatch in data ready command: expected {crc_calculated:#04x}, got {crc_received:#04x}")
            return False
        
        # data is ready if response == 1
        return (data_high_byte << 8) | data_low_byte == 1
