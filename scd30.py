'''
Sensor Driver for the Sensiron/Adafruit SCD30 sensor.

Includes:
    - an internal `_SCD30` class for i2c operations with no real abstraction
    - an `SCD30` class for simplified operations, intended for end users.

The `SCD30` class synchronizes using The `BusManager`, the `_SCD30` class does not.
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

# Command/Addr Constants
SCD30_ADDR = 0x61
SCD30_TRIGGER_CONTINUOUS_COMMAND                = 0x0010
SCD30_STOP_CONTINUOUS_COMMAND                   = 0x0104
SCD30_SET_MEASUREMENT_INTERVAL_COMMAND          = 0x4600
SCD30_GET_DATA_READY_COMMAND                    = 0x0202
SCD30_GET_READING_COMMAND                       = 0x0300
SCD30_GET_AND_SET_ASC_COMMAND                   = 0x5306 # See section 1.4.6 of the datasheet for more information 
SCD30_GET_AND_SET_FRC_COMMAND                   = 0x5204 # See section 1.4.6 of the datahseet for more information
SCD30_SET_AND_GET_TEMP_OFFSET_COMMAND           = 0x5403 # See section 1.4.7 of the datasheet for more information
SCD30_SET_AND_GET_ALTITUDE_COMPENSATION_COMMAND = 0x5102
SCD30_GET_FIRMWARE_VERSION_COMMAND              = 0xD100
SCD30_SOFT_RESET_COMMAND                        = 0xD304

logger = logging.getLogger(__name__)

# Dataclasses
@dataclass(frozen=True)
class SCD30Reading:
    CO2: float
    relative_humidity: float
    temperature: float

# Type alias for I2C bus, can be provided as int (`/dev/i2c-x`), str (full path), SMBus (will use current SMBus instance), None (will make best effort)
I2CBus = str | int | SMBus | None

class SCD30:
    '''
    High-level interface for using the Sensiron/Adafruit SCD30.
    
    Serializes concurrent I2C bus access with other devices using the BusManager.
    '''
    def __init__(self, bus: I2CBus, bus_manager: BusManager, bus_manager_number: int):
        self._SCD30 = _SCD30(bus)
        self._bus_manager = bus_manager
        self._bus_manager_number = bus_manager_number
        self._cached_reading: SCD30Reading | None = None

    # Helper methods (should not be called externally)
    def _refresh_cache(self):
        if self.data_ready:
            self._cached_reading = self._bus_manager.execute_on_bus(
                self._bus_manager_number,
                self._SCD30.get_reading
            )

    # Public API
    @property
    def data_ready(self):
        '''
        Returns true if new SCD30 data is ready.

        Note: The fastest polling interval supported by the SCD30 is 2 seconds.
        '''
        return self._bus_manager.execute_on_bus(
            self._bus_manager_number,
            self._SCD30.get_ready_status
        )

    @property
    def data_available(self):
        '''
        Alias for `SCD30.data_ready`.
        Kept so that the API surface is compatible with implementations using the Adafruit-circuitpython package.
        '''
        return self.data_ready

    @property
    def CO2(self) -> float:
        '''
        Returns the most recent CO2 measurment from the SCD30 sensor.
        '''
        # If new data is ready, refresh the cached reading
        self._refresh_cache()

        # If the cached reading is not None, return the CO2 value cached
        if self._cached_reading:
            return self._cached_reading.CO2
        
        logger.warning("CO2 data requested before cache could be filled. Returning 0.0...")
        return 0.0
    
    def get_CO2(self) -> float:
        '''
        Alias for `SCD30.CO2`
        '''
        return self.CO2

    @property
    def relative_humidity(self) -> float:
        '''
        Returns the most recent relative humidity value.
        '''
        self._refresh_cache()

        if self._cached_reading:
            return self._cached_reading.relative_humidity
        return 0.0

    @property
    def temperature(self) -> float:
        '''
        Returns the most recent temperature value (in degrees celcius).
        '''
        self._refresh_cache()

        if self._cached_reading:
            return self._cached_reading.temperature
        return 0.0

    @property
    def altitude(self) -> int:
        '''
        Returns the NDIR altitude compensation value from the SCD30 non-volatile memory. 
        '''
        logger.warning(
            "The SCD30's altitude compensation is for internal NDIR CO2 calculations.\n" +
            "This value should not be viewed as a source of truth for altitude."
        )
        return self._bus_manager.execute_on_bus(
            self._bus_manager_number,
            self._SCD30.get_altitude_compensation
        )
    
    @altitude.setter
    def altitude(self, altitude: int):
        '''
        Sets the NDIR altitude compensation in the SCD30 non-volatile memory.

        This value will persist between power cycles.
        '''
        self._bus_manager.execute_on_bus(
            self._bus_manager_number,
            self._SCD30.set_altitude_compensation,
            altitude
        )

    
    def set_ambient_pressure(self, ambient_pressure: int):
        '''
        Sets the ambient pressure used for NDIR calculations.

        **Note**: Pressure should be 0 (ignored) or 700 mBar <= pressure <= 1400 mBar

        **Implementation Note**: No get interface is offered because this value is provided at startup.
        It does not persist between runs, so it is not useful to keep an interface for and cache this data.
        '''
        self._bus_manager.execute_on_bus(
            self._bus_manager_number,
            self._SCD30.trigger_continuous_measurements,
            ambient_pressure
        )
    
    @property
    def frc(self) -> int:
        '''
        Returns the Force Recalibration Reference value (for CO2).
        '''
        return self._bus_manager.execute_on_bus(
            self._bus_manager_number,
            self._SCD30.get_frc_state
        )
    
    @frc.setter
    def frc(self, co2_reference: int):
        '''
        Sets the Forced Recalibration Reference value (for CO2).

        Note: 400 <= `co2_reference` <= 2000
        '''
        self._bus_manager.execute_on_bus(
            self._bus_manager_number,
            self._SCD30.set_frc_state,
            co2_reference
        )

    def set_measurement_interval(self, interval: int):
        '''
        Sets the SCD30's internal measurement interval.

        Note: 2 seconds <= interval <= 1800 seconds
        '''
        self._bus_manager.execute_on_bus(
            self._bus_manager_number,
            self._SCD30.set_measurement_interval,
            interval
        )
    
    def reset(self):
        '''
        Performs a soft-reset on the SCD30, any non-volatile memory (such as altitude,
        frc state, ASC state) will be unaffected.
        '''
        self._bus_manager.execute_on_bus(
            self._bus_manager_number,
            self._SCD30.run_soft_reset
        )
    
    @property
    def asc_enabled(self) -> bool:
        '''
        Automatic Self Recalibration state.
        '''
        return self._bus_manager.execute_on_bus(
            self._bus_manager_number,
            self._SCD30.get_asc_state
        )
    
    @asc_enabled.setter
    def asc_enabled(self, value: bool):
        '''
        Used to enable Automatic Self Recalibration (ASC). See section 1.4.6 of data sheet for more information.
        '''
        self._bus_manager.execute_on_bus(
            self._bus_manager_number,
            self._SCD30.set_asc_state,
            value
        )
    
    @property
    def temperature_offset(self) -> float:
        '''
        Returns the temperature offset in degrees celcius.

        See section 1.4.7 of the data sheet for more information.
        '''
        return self._bus_manager.execute_on_bus(
            self._bus_manager_number,
            self._SCD30.get_temperature_offset
        )
    
    @temperature_offset.setter
    def temperature_offset(self, offset: float):
        '''
        Setter for `SCD30.temperature_offset`.
        '''
        self._bus_manager.execute_on_bus(
            self._bus_manager_number,
            self._SCD30.set_temperature_offset,
            offset
        )
    
# Helper methods
def to_uint16(value: int) -> int:
    '''
    Clamps an integer to [0, 65535] range.

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
            f"CRC mismatch in second word: {hex(lmsb)}, {hex(llsb)}, intended crc: {hex(crc2)}, actual: {hex(scd30_data_crc8(bytes([lmsb, llsb])))}"
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

        elif isinstance(bus, SMBus):
            self.bus = bus

    def bus_exists(self) -> bool:
        return isinstance(self.bus, SMBus)
 
    def trigger_continuous_measurements(self, ambient_pressure: int | None):
        '''
        Triggers continuous measurement in the SCD30 sensor.

        Args:
            ambient_pressure (int | none): CO2 measurement will be compensated by this value. 
            ambient pressure should be 0 or [700, 1400] mBar. If 0 (or None), CO2 measurement will not
            be compensated using ambient pressure.
        '''
        if not self.bus_exists():
            raise RuntimeError("I2C Bus fails to exist, cannot read or write from I2C bus.")

        
        if ambient_pressure is None:
            ambient_pressure = 0
        
        if not (ambient_pressure == 0 or 700 <= ambient_pressure <= 1400):
            raise RuntimeError("ambient_pressure parameter must be 0 or [700, 1400] mBar.") 

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

        write_msg = i2c_msg.write(
            SCD30_ADDR,
            [
                cmd_high_byte, cmd_low_byte,
                pressure_high_byte, pressure_low_byte,
                crc
            ]
        )
        self.bus.i2c_rdwr(write_msg)

    
    def stop_continuous_measurements(self):
        '''
        Stops the continuous measurment of the SCD30
        '''
        if not self.bus_exists():
            logger.error("I2C Bus fails to exist, cannot read or write from I2C bus.")
            return
       
        cmd_high_byte, cmd_low_byte = uint16_to_two_bytes(SCD30_STOP_CONTINUOUS_COMMAND)

        write_msg = i2c_msg.write(SCD30_ADDR, [cmd_high_byte, cmd_low_byte])
        self.bus.i2c_rdwr(write_msg)

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

        write_msg = i2c_msg.write(
            SCD30_ADDR,
            [
                cmd_high_byte, cmd_low_byte,
                data_high_byte, data_low_byte,
                crc
            ]
        )
        self.bus.i2c_rdwr(write_msg)

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
        time.sleep(0.003)
        self.bus.i2c_rdwr(read_msg)
        
        data = list(read_msg)
        data_high_byte, data_low_byte, crc_received = data[0], data[1], data[2]

        crc_calculated = scd30_data_crc8(bytes([data_high_byte, data_low_byte]))

        if crc_calculated != crc_received:
            logger.error(f"CRC mismatch in data ready command: expected {crc_calculated:#04x}, got {crc_received:#04x}")
            return False
        
        # data is ready if response == 1
        return (data_high_byte << 8) | data_low_byte == 1

    def get_reading(self) -> SCD30Reading:
        '''
        Gets a (CO2, Relative Humidity, Temperature) reading from the SCD30 sensor.

        `sensor.get_ready_status()` should be run before trying to get data.

        This method is particularly complex, to see how field decoding works check the 
        data sheet for the SCD30 provided by Sensiron.
        '''
        if not self.bus_exists():
            raise RuntimeError("I2C Bus fails to exist, cannot read or write from I2C bus.")
            

        cmd_high_byte, cmd_low_byte = uint16_to_two_bytes(SCD30_GET_READING_COMMAND)

        write_msg = i2c_msg.write(SCD30_ADDR, [cmd_high_byte, cmd_low_byte])
        read_msg = i2c_msg.read(SCD30_ADDR, 18)

        self.bus.i2c_rdwr(write_msg)
        time.sleep(0.003)
        self.bus.i2c_rdwr(read_msg)

        read_data = list(read_msg)

        co2_raw = read_data[0:6]
        temperature_raw = read_data[6:12]
        humidity_raw = read_data[12:18]
        
        co2 = parse_float_with_crc(co2_raw)
        temperature = parse_float_with_crc(temperature_raw)
        humidity = parse_float_with_crc(humidity_raw)

        if math.isnan(co2) or math.isnan(humidity) or math.isnan(temperature):
            raise RuntimeError("One or more reading variables are NaN, reduce polling frequency if this happens often")

        return SCD30Reading(
            CO2               = co2,
            relative_humidity = humidity,
            temperature       = temperature
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
        time.sleep(0.003)
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
        
        cmd_msb, cmd_lsb = uint16_to_two_bytes(SCD30_GET_AND_SET_ASC_COMMAND)
        state = int(state) & 0xFF # convert bool->int, and mask such that we only take lower 8-bits (this is almost certainly unnecessary)
        crc = scd30_data_crc8(bytes([0x00, state]))

        write_msg = i2c_msg.write(
            SCD30_ADDR,
            [
                cmd_msb,
                cmd_lsb,
                0x00, # msb of data section is always 0x00.
                state,
                crc
            ]
        )

        self.bus.i2c_rdwr(write_msg)

    def get_frc_state(self) -> int:
        '''
        Returns the current Forced Recalibration (FRC) value for CO2 in ppm.

        By default there will not be an FRC value. See section 1.4.6 in the datasheet for more information.
        '''
        if not self.bus_exists():
            raise RuntimeError("I2C Bus fails to exist, cannot read or write from I2C bus.")
        
        cmd_high_byte, cmd_low_byte = uint16_to_two_bytes(SCD30_GET_AND_SET_FRC_COMMAND)
        write_msg = i2c_msg.write(SCD30_ADDR, [cmd_high_byte, cmd_low_byte])
        read_msg = i2c_msg.read(SCD30_ADDR, 3)

        self.bus.i2c_rdwr(write_msg)
        time.sleep(0.003)
        self.bus.i2c_rdwr(read_msg)

        msb, lsb, crc = list(read_msg)

        calculated_crc = scd30_data_crc8(bytes([msb, lsb]))
        if crc != calculated_crc:
            logger.error(
                f"CRC mismatch in word {msb:#04x}, {lsb:#04x}" +
                f" received CRC: {crc:#04x}, computed CRC: {calculated_crc:#04x}"
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

    def get_temperature_offset(self) -> float:
        '''
        Returns the temperature offset stored in the SCD30 non-volatile memory.

        The sensor naturally generates heat, so this exists to allow users to run the sensor in its intended
        environment for some period of time, then compare the measured temperature to a trusted thermometer, and
        determine the difference.
         
        Having correct temperature is key for measuring relative humiditiy.

        Note: Internally the offset is stored as [offset (in celcius) * 100]. This method, returns the offset, so we divide by 100.
        '''
        if not self.bus_exists():
            raise RuntimeError("I2C Bus fails to exist, cannot read or write from I2C bus.")
        
        cmd_high_byte, cmd_low_byte = uint16_to_two_bytes(SCD30_SET_AND_GET_TEMP_OFFSET_COMMAND)
        
        write_msg = i2c_msg.write(SCD30_ADDR, [cmd_high_byte, cmd_low_byte])
        read_msg = i2c_msg.read(SCD30_ADDR, 3)

        self.bus.i2c_rdwr(write_msg)
        time.sleep(0.003)
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
            raise RuntimeError("I2C Bus fails to exist, cannot read or write from I2C bus.")
        
        cmd_high_byte, cmd_low_byte = uint16_to_two_bytes(SCD30_SET_AND_GET_TEMP_OFFSET_COMMAND)

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

    def get_altitude_compensation(self) -> int:
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
        time.sleep(0.003)
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
        time.sleep(0.003)
        self.bus.i2c_rdwr(read_msg)

        msb, lsb, crc = list(read_msg)

        computed_crc = scd30_data_crc8(bytes([msb, lsb]))
        if computed_crc != crc:
            raise RuntimeError(
                f"CRC mismatch on {msb:#04x}, {lsb:#04x}\n * Received: {crc:#04x}, Computed: {computed_crc:#04x}"
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
        
        write_msg = i2c_msg.write(SCD30_ADDR, [cmd_msb, cmd_lsb])
        self.bus.i2c_rdwr(write_msg)