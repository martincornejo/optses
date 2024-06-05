from optses.application.abstract_application import AbstractApplication

import pyomo.environ as opt

class Arbitrage(AbstractApplication):
    def __init__(self) -> None:
        self.name = "arbitrage"

    def build(self, block: opt.Model) -> None:
        model = block.model()

        @block.Expression()
        def cost(b):
            return sum((model.grid[t] - model.feedin[t]) * model.price[t] * model.dt for t in model.time)
