from abc import ABC, abstractmethod

from pyomo.environ import Model

class AbstractApplication(ABC):
    @abstractmethod
    def build(self, model: Model) -> None:
        pass

    @property
    @abstractmethod
    def profile(self):
        pass

    # TODO: add abtract method indicating that the `power` variable interface is required
