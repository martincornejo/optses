from optses.storage.abstract_storage import AbstractStorageModel
from optses.converter.converter import AbstractConverter
from optses.converter.converter import IdealConverter

class StorageSystem:
    def __init__(
        self,
        cell_model: AbstractStorageModel,
        converter_model: AbstractConverter = None,
    ) -> None:
        if converter_model is None:
            converter_model = IdealConverter()

        self._cell_model = cell_model
        self._converter_model = converter_model

    def build(self, block) -> None:
        self._cell_model.build(block)
        self._converter_model.build(block)
