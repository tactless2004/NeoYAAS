'''
busmanager.py serializes I2C access in a multithreaded environment.
'''

import logging
from multiprocessing import Lock
from typing import Callable
logger = logging.getLogger(__name__)
logging.basicConfig(filename="neoyaas.log", level=logging.INFO)


class BusManager:
    '''
    BusManager maintains a list of n locks used for serializing multithreded access on the I2C busses  
    '''
    def __init__(self, n_busses: list[int]):
        self.locks = {
            k : Lock()
            for k in n_busses
        }

    def sensor_reading(self, bus_number: int, calling_function: Callable):
        # First verify the bus is registered with the BusManager
        if bus_number not in self.locks:
            logger.error(f"I2C Bus Number {bus_number} is not registered with this BusManager.")
            return
        
        # Second verify the bus is registered and that the lock exists.
        lock_ref = self.locks.get(bus_number)
        if lock_ref is None:
            logger.error(
                f"I2C Bus Number {bus_number} is registered with this BusManager, but the Lock is None."
            )
            return
        
        # Finally run the calling function with the bus-correct lock.
        with lock_ref:
            return calling_function()

    

