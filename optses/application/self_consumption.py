from optses.application.abstract_application import AbstractApplication

import numpy as np
import pandas as pd
import pyomo.environ as opt

class SelfConsumptionIncrease(AbstractApplication):
    def __init__(self, 
        electricity_price: float, 
        feedin_tariff: float, 
        load_profile: pd.Series, 
        generation_profile: pd.Series = None
    ) -> None:
        self._electricity_price = electricity_price
        self._feedin_tariff = feedin_tariff

        if generation_profile is not None:
            self._residual_load = load_profile - generation_profile
        else:
            self._residual_load = load_profile

        self.name = "self_consumption"

    @property
    def profile(self):
        return self._residual_load

    def build(self, block: opt.Block) -> None:
        self.param(block)
        self.variable_grid(block)
        self.objective_cost(block)

    def param(self, block):
        model = block.model()

        block.power_profile      = opt.Param(model.time, within=opt.Reals, initialize=lambda b, t: self._residual_load.iloc[t])
        block.electricity_price  = opt.Param(within=opt.NonNegativeReals, initialize=self._electricity_price)
        block.feedin_tariff      = opt.Param(within=opt.NonNegativeReals, initialize=self._feedin_tariff)
        # TODO: time-varying prices
        # block.electricity_price = opt.Param(model.time, within=opt.NonNegativeReals)
        # block.feedin_tariff     = opt.Param(model.time, within=opt.NonNegativeReals)

    def variable_grid(self, block):
        model = block.model()
        block.grid   = opt.Var(model.time, within=opt.NonNegativeReals)
        block.feedin = opt.Var(model.time, within=opt.NonNegativeReals)

        @block.Expression(model.time)
        def power(b, t):
            return b.grid[t] - b.feedin[t]

    # TODO: implement feedin limit
    # def constraint_feedin_limit(self, mode):
    #     if model.feedin_limit is None:
    #         return

    #     @model.Constraint(model.time)
    #     def feedin_limit(m, t):
    #         return m.feedin[t] <= m.feedin_limit

    def objective_cost(self, block):
        model = block.model()

        @block.Expression()
        def cost(b):
            return sum(b.grid[t] * b.electricity_price - b.feedin[t] * b.feedin_tariff for t in model.time) * model.dt

    def recover_results(self, block):
        model = block.model()
        return {
            "grid":   np.array([opt.value(block.grid[t])   for t in model.time]),
            "feedin": np.array([opt.value(block.feedin[t]) for t in model.time]),
        }
