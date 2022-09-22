from abc import ABC, abstractmethod

from pyomo.environ import Model

class AbstractApplication(ABC):
    @abstractmethod
    def build(self, model: Model) -> None:
        pass

    @abstractmethod
    def recover_results(self, block) -> dict:
        pass
