'''
busmanager.py serializes I2C access in a multithreaded environment.
'''

import logging
from threading import Lock
from typing import Callable, TypeVar, ParamSpec

logger = logging.getLogger(__name__)

# For generic typing
P = ParamSpec("P")
R = TypeVar("R")
class BusManager:
    '''
    BusManager maintains a list of n locks used for serializing multithreded access on the I2C busses  
    '''
    def __init__(self, bus_ids: list[int]):
        if bus_ids is None or not all(n >= 0 for n in bus_ids):
            raise RuntimeError(f"Invalid I2C bus IDs: {bus_ids}.")

        self.locks = {
            k : Lock()
            for k in bus_ids
        }

    def execute_on_bus(
            self,
            bus_number: int,
            calling_function: Callable[P, R],
            *args: P.args,
            **kwargs: P.kwargs
    ) -> R:
        
        # First verify the bus is registered with the BusManager
        if bus_number not in self.locks:
            raise RuntimeError(f"I2C Bus Number {bus_number} is not registered with this BusManager.")
        
        # Second verify the bus is registered and that the lock exists.
        lock_ref = self.locks.get(bus_number)
        if lock_ref is None:
            raise RuntimeError(
                f"I2C Bus Number {bus_number} is registered with this BusManager, but the Lock is None."
            )
        
        # Finally run the calling function with the bus-correct lock.
        with lock_ref:
            return calling_function(*args, **kwargs)

    

