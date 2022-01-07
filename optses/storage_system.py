from optses.storage.abstract_storage import AbstractStorageModel
from optses.converter.converter import AbstractConverter
from optses.converter.converter import IdealConverter


class StorageSystem:
    def __init__(self, name: str, cell_model: AbstractStorageModel, converter_model: AbstractConverter = None) -> None:
        if converter_model is None:
            converter_model = IdealConverter()
        
        self._name = name
        self._cell_model = cell_model
        self._converter_model = converter_model
    
    @property
    def name(self):
        return self._name

    def build(self, block):
        self._cell_model.build(block)
        self._converter_model.build(block)
    
    # def update_model(self, model, data):
    #     self._cell_model.update_model(model, data)
    #     self._converter_model.update_model(model, data)

    def recover_results(self, block):
        return {
            **self._cell_model.recover_results(block),
            **self._converter_model.recover_results(block)
        }

