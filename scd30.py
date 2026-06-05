'''
Sensor Driver for the Sensiron/Adafruit SCD30 sensor.

Includes:
    - an internal `_SCD30` class for i2c operations with no real abstraction
    - an `SCD30` class for simplified operations, intended for end users.

The `SCD30` class synchronizes using The `BusManager` by default, the `_SCD30` class does not.
'''

# python std library
import time
import math
import struct
import logging
from dataclasses import dataclass

# third part imports
from smbus2 import SMBus, i2c_msg

# local imports
from busmanager import BusManager

logger = logging.getLogger(__name__)

# Dataclasses
@dataclass(frozen=True)
class SCD30Reading:
    CO2: float
    relative_humidity: float
    temperature: float

I2CBus = str | int | SMBus | None

class SCD30:
    def __init__(self, bus: I2CBus, bus_manager: BusManager, bus_manager_number: int):
        self._SCD30 = _SCD30(I2CBus)

# Constants
SCD30_ADDR = 0x61
SCD30_TRIGGER_CONTINUOUS_COMMAND = 0x0010
SCD30_STOP_CONTINUOUS_COMMAND = 0x0104
SCD30_SET_MEASUREMENT_INTERVAL_COMMAND = 0x4600
SCD30_GET_DATA_READY_COMMAND = 0x0202
SCD30_GET_READING_COMMAND = 0x0300
SCD30_GET_AND_SET_ASC_COMMAND = 0x5306 # See section 1.4.6 of the datasheet for more information 
SCD30_GET_AND_SET_FRC_COMMAND = 0x5204 # See section 1.4.6 of the datahseet for more information
SCD30_SET_AND_GET_TEMP_OFFSET_COMMMAND = 0x5403 # See section 1.4.7 of the datasheet for more information
SCD30_SET_AND_GET_ALTITUDE_COMPENSATION_COMMAND = 0x5102
SCD30_GET_FIRMWARE_VERSION_COMMAND = 0xD100
SCD30_SOFT_RESET_COMMAND = 0xD304

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

# Low-level implementation
class _SCD30:
    def __init__(self, bus: I2CBus):
        self.bus: SMBus # forward definition for type hinting

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

    def get_reading(self) -> SCD30Reading | None:
        '''
        Gets a (CO2, Relative Humidity, Temperature) reading from the SCD30 sensor.

        `sensor.get_ready_status()` should be run before trying to get data.

        This method is particularly complex, to see how field decoding works check the 
        data sheet for the SCD30 provided by Sensiron.
        '''
        if not self.bus_exists():
            logger.error("I2C Bus fails to exist, cannot read or write from I2C bus.")
            return

        cmd_high_byte, cmd_low_byte = uint16_to_two_bytes(SCD30_GET_READING_COMMAND)

        write_msg = i2c_msg.write(SCD30_ADDR, [cmd_high_byte, cmd_low_byte])
        read_msg = i2c_msg.read(SCD30_ADDR, 18)

        self.bus.i2c_rdwr(write_msg)
        self.bus.i2c_rdwr(read_msg)

        read_data = list(read_msg)

        co2_raw = read_data[0:6]
        humidity_raw = read_data[6:12]
        temperature_raw = read_data[12:18]
        
        co2 = parse_float_with_crc(co2_raw)
        humidity = parse_float_with_crc(humidity_raw)
        temperature = parse_float_with_crc(temperature_raw)

        if math.isnan(co2) or math.isnan(humidity) or math.isnan(temperature):
            logger.error("One or more reading variables are NaN, reduce polling frequency if this happens often")
            return

        return SCD30Reading(
            co2,
            humidity,
            temperature
        )
    
    def get_asc_state(self) -> bool:
        '''
        Returns the state of the ASC routine (true => running, false => not running).

        By default the SCD30 performs an automatic self calibration routine (ASC).

        This takes 7 days to properly calibrate, during which the sensor must be exposed to air for at
        least 1 hour a day.

        If ASC is deactivated, the sensor will still use its parameters from a previous ASC run.

        See section 1.4.6 of the datasheet for more information.
        '''
        if not self.bus_exists():
            logger.error("I2C Bus fails to exist, cannot read or write from I2C bus.")
            return False
        
        cmd_high_byte, cmd_low_byte = uint16_to_two_bytes(SCD30_GET_AND_SET_ASC_COMMAND)
        write_msg = i2c_msg.write(SCD30_ADDR, [cmd_high_byte, cmd_low_byte])

        read_msg = i2c_msg.read(SCD30_ADDR, 3)

        self.bus.i2c_rdwr(write_msg)
        self.bus.i2c_rdwr(read_msg)

        raw_data = list(read_msg)

        msb, lsb, crc = raw_data[0], raw_data[1] , raw_data[2]

        calculated_crc = scd30_data_crc8(bytes([msb, lsb]))
        if calculated_crc != crc:
            logger.error(f"CRC mismatch: crc received = {crc:#04x}, crc computed = {calculated_crc:#04x}")
            return False
        
        return (msb << 8) | lsb == 1
    
    def set_asc_state(self, state = True):
        '''
        Sets ASC state to `state`.

        Automatic self-calibration (ASC) is run by the SCD30 by default, but you might want to 
        deactivate it for some reason. See section 1.4.6 of the datasheet for info on ASC. 
        '''
        if not self.bus_exists():
            logger.error("I2C Bus fails to exist, cannot read or write from I2C bus.")
            return False
        
        write_msg = i2c_msg.write(
            SCD30_ADDR,
            [
                (SCD30_GET_AND_SET_ASC_COMMAND >> 8) & 0xFF,
                (SCD30_GET_AND_SET_ASC_COMMAND & 0xFF),
                0x00,
                int(state) & 0xFF,
                scd30_data_crc8(bytes([0x00, int(state) & 0xFF]))
            ]
        )

        self.bus.i2c_rdwr(write_msg)

    def get_frc_state(self) -> int:
        '''
        Returns the current Forced Recalibration (FRC) value for CO2 in ppm.

        By default there will not be an FRC value. See section 1.4.6 in the datasheet for more information.
        '''
        if not self.bus_exists():
            logger.error("I2C Bus fails to exist, cannot read or write from I2C bus.")
            return False
        
        cmd_high_byte, cmd_low_byte = uint16_to_two_bytes(SCD30_GET_AND_SET_FRC_COMMAND)
        write_msg = i2c_msg.write(SCD30_ADDR, [cmd_high_byte, cmd_low_byte])
        read_msg = i2c_msg.read(SCD30_ADDR, 3)

        self.bus.i2c_rdwr(write_msg)
        self.bus.i2c_rdwr(read_msg)

        msb, lsb, crc = list(read_msg)

        calculated_crc = scd30_data_crc8(bytes([msb, lsb]))
        if crc != calculated_crc:
            logger.error(
                f"CRC mismatch in word {msb:#04x}, {lsb:#04x}" +
                f" received CRC: {crc:#04x}, computed CRC: {crc:#04x}"
            )
            return -1
        
        return (msb << 8) | lsb


    def set_frc_state(self, frc_state: int):
        '''
        Sets the Force Recalibration value for the SCD30 to `frc_state`.

        By default the SCD30 automatically calibrates, however we can manually set the reference
        concentration for the CO2 calibration curve.

        Note: 400 ppm <= CO2_{ref} <= 2000 ppm 
        '''
        if not self.bus_exists():
            logger.error("I2C Bus fails to exist, cannot read or write from I2C bus.")
            return
        
        if not 400 <= frc_state <= 2000:
            logger.error(f"CO2 reference concentration must be 400 ppm <= concentration <= 2000 ppm.")
            return
        
        cmd_high_byte, cmd_low_byte = uint16_to_two_bytes(SCD30_GET_AND_SET_FRC_COMMAND)
        data_high_byte, data_low_byte = uint16_to_two_bytes(to_uint16(frc_state))
        crc = scd30_data_crc8(bytes([data_high_byte, data_low_byte]))

        write_msg = i2c_msg.write(
            SCD30_ADDR,
            [
                cmd_high_byte,
                cmd_low_byte,
                data_high_byte,
                data_low_byte,
                crc
            ]
        )

        self.bus.i2c_rdwr(write_msg)

    def get_temperature_offset(self) -> float | None:
        '''
        Returns the temperature offset stored in the SCD30 non-volatile memory.

        The sensor naturally generates heat, so this exists to allow users to run the sensor in its intended
        environment for some period of time, then compare the measured temperature to a trusted thermometer, and
        determine the difference.
         
        Having correct temperature is key for measuring relative humiditiy.

        Note: Internally the offset is stored as [offset (in celcius) * 100]. This method, returns the offset, so we divide by 100.
        '''
        if not self.bus_exists():
            logger.error("I2C Bus fails to exist, cannot read or write from I2C bus.")
            return
        
        cmd_high_byte, cmd_low_byte = uint16_to_two_bytes(SCD30_SET_AND_GET_TEMP_OFFSET_COMMMAND)
        
        write_msg = i2c_msg.write(SCD30_ADDR, [cmd_high_byte, cmd_low_byte])
        read_msg = i2c_msg.read(SCD30_ADDR, 3)

        self.bus.i2c_rdwr(write_msg)
        self.bus.i2c_rdwr(read_msg)

        msb, lsb, crc = list(read_msg)
        computed_crc = scd30_data_crc8(bytes([msb, lsb]))
        if computed_crc != crc:
            raise RuntimeError(
                f"CRC mismatch on data: {msb:#04x}, {lsb:#04x}\n * CRC Received: {crc:#04x}, CRC Computed: {computed_crc:#04x}"
            )
        
        return ((msb << 8) | lsb) / 100 # see note in docstring for why divide by 100

    def set_temperature_offset(self, offset: float):
        '''
        Set temperature offset to `offset` to account for heat generated by the sensor itself.
        
        See section 1.4.7 of the datasheet for more information.
        '''
        if not self.bus_exists():
            logger.error("I2C Bus fails to exist, cannot read or write from I2C bus.")
            return
        
        cmd_high_byte, cmd_low_byte = uint16_to_two_bytes(SCD30_SET_AND_GET_TEMP_OFFSET_COMMMAND)

        # offset format is temperature in celcius * 100, so 1.8 celcius = 180 offset.
        offset *= 100
        offset = int(offset) # explicitly cast as int for type checkers

        data_high_byte, data_low_byte = uint16_to_two_bytes(offset)
        crc = scd30_data_crc8(bytes([data_high_byte, data_low_byte]))

        write_msg = i2c_msg.write(
            SCD30_ADDR,
            [
                cmd_high_byte,
                cmd_low_byte,
                data_high_byte,
                data_low_byte,
                crc
            ]
        )

        self.bus.i2c_rdwr(write_msg)

    def get_altitude_compensation(self) -> int | None:
        '''
        Returns altitude compensation in meters above sea level.

        See section 1.4.8 of the data sheet for why this is important for CO2 concentration calcualtions.
        '''
        if not self.bus_exists():
            raise RuntimeError("I2C Bus fails to exist, cannot read or write from I2C bus.")

        cmd_high_byte, cmd_low_byte = uint16_to_two_bytes(
            SCD30_SET_AND_GET_ALTITUDE_COMPENSATION_COMMAND
        )
        write_msg = i2c_msg.write(SCD30_ADDR, [cmd_high_byte, cmd_low_byte])
        read_msg = i2c_msg.read(SCD30_ADDR, 3)

        self.bus.i2c_rdwr(write_msg)
        self.bus.i2c_rdwr(read_msg)

        msb, lsb, crc = list(read_msg)
        computed_crc = scd30_data_crc8(bytes([msb, lsb]))

        if computed_crc != crc:
            raise RuntimeError(
                f"CRC mistmatch on {msb:#04x}, {lsb:#04x}.\n * Received: {crc:#04x}, Computed: {computed_crc:04x}"
            )
        
        return (msb << 8) | lsb
    

    def set_altitude_compensation(self, altitude_comp: int):
        '''
        Sets the SCD30 altitude compensation to `altitude_comp` meters above sea level.
        '''
        if not self.bus_exists():
            logger.error("I2C Bus fails to exist, cannot read or write from I2C bus.")
            return
        
        cmd_high_byte, cmd_low_byte = uint16_to_two_bytes(
            SCD30_SET_AND_GET_ALTITUDE_COMPENSATION_COMMAND
        )
        data_high_byte, data_low_byte = uint16_to_two_bytes(
            to_uint16(altitude_comp)
        )
        crc = scd30_data_crc8(bytes([data_high_byte, data_low_byte]))

        write_msg = i2c_msg.write(
            SCD30_ADDR,
            [
                cmd_high_byte,
                cmd_low_byte,
                data_high_byte,
                data_low_byte,
                crc
            ]
        )

        self.bus.i2c_rdwr(write_msg)

    def get_firmware_version(self) -> str:
        '''
        Returns the firmware version for the SCD30 in `major_version`.`minor_version` format.
        '''
        if not self.bus_exists():
            raise RuntimeError("I2C Bus fails to exist, cannot read or write from I2C bus.")
           
        cmd_msb, cmd_lsb = uint16_to_two_bytes(SCD30_GET_FIRMWARE_VERSION_COMMAND)
        
        write_msg = i2c_msg.write(SCD30_ADDR, [cmd_msb, cmd_lsb])
        read_msg = i2c_msg.read(SCD30_ADDR, 3)

        self.bus.i2c_rdwr(write_msg)
        self.bus.i2c_rdwr(read_msg)

        msb, lsb, crc = list(read_msg)

        computed_crc = scd30_data_crc8(bytes([msb, lsb]))
        if computed_crc != crc:
            raise RuntimeError(
                f"CRC mismatch on {msb:#04x}, {lsb:#04x}\n * Received: {crc:#04x}, Computed: {crc:#04x}"
            )
        
        return f"{msb}.{lsb}"

    def run_soft_reset(self):
        '''
        Runs a soft-reset which power cycles the system controller.

        Note: This does not affect the ASC or FRC data, or anything else
        stored in non-volatile memory.
        '''
        if not self.bus_exists():
            logger.error("I2C Bus fails to exist, cannot read or write from I2C bus.")
            return
        
        cmd_msb, cmd_lsb = uint16_to_two_bytes(SCD30_SOFT_RESET_COMMAND)
        
        self.bus.write_byte_data(
            SCD30_ADDR,
            cmd_msb,
            cmd_lsb
        )