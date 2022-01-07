import numpy as np
import pandas as pd
import pyomo.environ as opt

from optses.application.abstract_application import AbstractApplication

class PeakShaving(AbstractApplication):
    def __init__(self, load_profile: pd.Series, peak_power_price:float, electricity_price=None) -> None:
        if electricity_price is None:
            electricity_price = 0.0

        self._power_profile = load_profile 
        self._peak_power_price = peak_power_price
        self._electricity_price = electricity_price

        self.requires_feedin = False
        self.name = "peak_shaving"

    @property
    def profile(self):
        return self._power_profile

    # def init_electricity_price(self, m, t): # TODO: better implement this as a closure?
    #     if isinstance(self._electricity_price, float):
    #         return self._electricity_price
    #     return self._electricity_price[t]    # array like (pd.Series, np.array)

    def build(self, block: opt.Block) -> None:
        self.param(block)

        self.variable_grid_power(block)
        self.variable_peak_power(block)

        self.constraint_peak_power(block)

        self.objective_cost(block)

    def param(self, block):
        model = block.model()

        block.power_profile     = opt.Param(model.time, within=opt.Reals, mutable=True, initialize=lambda b, t: self._power_profile.iloc[t])
        block.peak_power_price  = opt.Param(within=opt.NonNegativeReals, mutable=True, initialize=self._peak_power_price)
        block.electricity_price = opt.Param(within=opt.NonNegativeReals, mutable=True, initialize=self._electricity_price)
        # model.electricity_price = opt.Param(model.time, within=opt.NonNegativeReals, mutable=True, initialize=self.init_electricity_price)

    # TODO: make a class to build grid model?
    # def grid_variable(self, model): # make block?
    #     model.grid   = opt.Var(model.time, within=opt.NonNegativeReals)
    #     model.feedin = opt.Var(model.time, within=opt.NonNegativeReals)
    #     if not self._application_model.requires_feedin:
    #         model.feedin.fix(0.0)
    def variable_grid_power(self, block):
        model = block.model()
        block.grid = opt.Var(model.time, within=opt.NonNegativeReals)

        @block.Expression(model.time)
        def power(b, t):
            return b.grid[t]

    def variable_peak_power(self, block):
        block.peak = opt.Var(within=opt.NonNegativeReals)

    def constraint_peak_power(self, block):
        model = block.model()

        @block.Constraint(model.time)
        def peak_power_constraint(m, t):
            return m.grid[t] <= m.peak

    def objective_cost(self, block):
        model = block.model()
        
        @block.Expression()
        def cost(b):
            return b.peak * b.peak_power_price + sum(b.grid[t] * b.electricity_price for t in model.time)

    def recover_results(self, block):
        model = block.model()
        return {
            "grid": np.array([opt.value(block.grid[t]) for t in model.time]),
            "peak": opt.value(block.peak)
        }

