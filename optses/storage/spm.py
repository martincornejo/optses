from optses.storage.abstract_storage import AbstractStorageModel

class SingleParticleModel(AbstractStorageModel):
    def __init__(self) -> None:
        ...

    def build(self, block) -> None:
        ...

    def recover_results(self, block) -> dict:
        return dict()