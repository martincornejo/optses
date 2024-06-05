from optses.application.abstract_application import AbstractApplication

import numpy as np
import pandas as pd
import pyomo.environ as opt

class SelfConsumptionIncrease(AbstractApplication):
    def __init__(self, 
        electricity_price: float, 
        feedin_tariff: float, 
        feedin_limit: float = None
    ) -> None:
        self._electricity_price = electricity_price
        self._feedin_tariff = feedin_tariff
        self._feedin_limit = feedin_limit

        self.name = "self_consumption" # TODO: name as input argument

    def build(self, block: opt.Model) -> None:
        model = block.model()

        block.electricity_price  = opt.Param(within=opt.NonNegativeReals, initialize=self._electricity_price)
        block.feedin_tariff      = opt.Param(within=opt.NonNegativeReals, initialize=self._feedin_tariff)

        # TODO: implement feedin_limit
        # if self._feedin_limit:
        # param feedin limit
        # curtailment: add new variable and recover results
        # @block.Constraint(model.time)
        # def feedin_limit_constraint(m, t):
        #     return model.feedin[t] <= m.feedin_limit

        @block.Expression()
        def cost(b):
            return sum(model.grid[t] * b.electricity_price - model.feedin[t] * b.feedin_tariff for t in model.time) * model.dt
        
        # TODO: time-varying prices ?

