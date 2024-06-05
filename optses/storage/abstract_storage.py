from abc import ABC, abstractmethod

from pyomo.core.base.PyomoModel import Model

class AbstractStorageModel(ABC):

    @abstractmethod
    def build(self, block) -> None:
        pass

