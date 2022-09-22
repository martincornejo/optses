import pyomo.environ as opt

from abc import ABC, abstractmethod
from pyomo.environ import Model

from optses.storage.abstract_storage import AbstractStorageModel
from optses.converter.converter import AbstractConverter
from optses.converter.converter import IdealConverter

class AbstractStorageSystem(ABC):
    @abstractmethod
    def build(self, model: Model) -> None:
        pass

    @abstractmethod
    def recover_results(self, model: Model) -> dict:
        pass


class StorageSystem(AbstractStorageSystem):
    def __init__(self, name: str, cell_model: AbstractStorageModel, converter_model: AbstractConverter = None) -> None:
        if converter_model is None:
            converter_model = IdealConverter()
        
        self.name = name
        self._cell_model = cell_model
        self._converter_model = converter_model

    def build(self, block) -> None:
        self._cell_model.build(block)
        self._converter_model.build(block)

    def recover_results(self, block) -> dict:
        return {
            **self._cell_model.recover_results(block),
            **self._converter_model.recover_results(block)
        }

class MultiStorageSystem(AbstractStorageSystem):
    def __init__(self, *systems) -> None:
        self._systems = systems

    def build(self, block) -> None:
        model = block.model()

        # build all subsystems
        for storage in self._systems:
            block.add_component(storage.name, opt.Block(rule=storage.build))

        @block.Expression(model.time)
        def power(b, t):
            return sum(b.find_component(system.name).power[t] for system in self._systems)

        @block.Expression()
        def cost(b):
            return sum(model.find_component(system.name).cost for system in self._systems)

    def recover_results(self, block) -> dict:
        return {system.name: system.recover_results(block) for system in self._systems}
