from dataclasses import dataclass
from pandas import DataFrame

# TODO: use dataclasses to easily parametrize models?

@dataclass
class StorageData:
    name: str
    capacity: float
    power: float
    efficiency: float
    soc_bound: tuple = (0., 1.)
    soc_init: float = 0.5

# @dataclass
# class PowerElectronics:
#     power: float

@dataclass
class ModelData:
    storage: StorageData
    # power_electronics: Optional[PowerElectronics] = None
    timeseries: DataFrame

