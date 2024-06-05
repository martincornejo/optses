import numpy as np
import pandas as pd
import pyomo.environ as opt

from optses.application.abstract_application import AbstractApplication

class PeakShaving(AbstractApplication):
    def __init__(self, peak_power_price:float, electricity_price=0.0, peak_power_min=0.0) -> None:
        self._peak_power_price = peak_power_price
        self._electricity_price = electricity_price
        self._peak_power_min = peak_power_min

        self.name = "peak_shaving" # TODO: name as input

    def build(self, block: opt.Model) -> None:
        model = block.model()
        block.electricity_price = opt.Param(within=opt.NonNegativeReals, mutable=True, initialize=self._electricity_price)
        block.peak_power_price  = opt.Param(within=opt.NonNegativeReals, mutable=True, initialize=self._peak_power_price)
        block.peak_power_min    = opt.Param(within=opt.NonNegativeReals, mutable=True, initialize=self._peak_power_min)

        block.peak = opt.Var(within=opt.NonNegativeReals, bounds=(block.peak_power_min, None))

        @block.Constraint(model.time)
        def peak_power_constraint(b, t):
            return model.grid[t] <= b.peak

        # TODO: check and differentiate if peak_power_price == 0? Probably not, else also handle price param
        @block.Expression()
        def cost(b):
            return b.peak * b.peak_power_price + sum(model.grid[t] * b.electricity_price for t in model.time) * model.dt
