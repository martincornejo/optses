import pyomo.environ as opt
import numpy as np

from pyomo.core.base.PyomoModel import Model
from optses.storage.abstract_storage import AbstractStorageModel

class EnergyReservoirModel(AbstractStorageModel):
    """Class to construct storage model"""
    def __init__(self, capacity, power, soc_init=None) -> None:
        self._capacity = capacity
        self._power = power

        # handle optional keyword arguments?
        # if soc_init is None:
        #   ...

    def build(self, block) -> None:
        self.variable_power(block)
        self.variable_soc(block)

        self.constraint_soc(block)

        self.objective_cost(block)

    def variable_power(self, block):
        model = block.model()
        ## Params
        block.max_power = opt.Param(
            within=opt.NonNegativeReals,
            initialize=self._power,
            mutable=True
        )
        ## Variables
        block.pc = opt.Var(model.time, bounds=(0, block.max_power))
        block.pd = opt.Var(model.time, bounds=(0, block.max_power))

        @block.Expression(model.time)
        def power_dc(b, t):
            return b.pc[t] - b.pd[t]


    def variable_soc(self, block):
        model = block.model()
        ## Params
        block.capacity = opt.Param(
            within=opt.NonNegativeReals,
            initialize=self._capacity,
            mutable=True
        )
        block.soc_min = opt.Param(
            within=opt.NonNegativeReals,
            default=0.0,
            mutable=True
        )
        block.soc_max = opt.Param(
            within=opt.NonNegativeReals,
            default=1.0,
            mutable=True
        )
        block.soc_start = opt.Param(
            within=opt.NonNegativeReals,
            default=0.5,
            mutable=True
        )

        ## Variables
        block.soc = opt.Var(model.time, 
            bounds=(
                block.soc_min*block.capacity, 
                block.soc_max*block.capacity
            )
        )


    def constraint_soc(self, block):
        model = block.model()
        
        ## Params
        block.effc = opt.Param(
            within=opt.PercentFraction,
            default=0.97,
            mutable=True 
        )
        block.effd = opt.Param(
            within=opt.PercentFraction,
            default=0.97,
            mutable=True 
        )
        block.psd = opt.Param(
            within=opt.NonNegativeReals,
            default=0.0,
            mutable=True
        )

        ## Constraints
        @block.Constraint(model.time)
        def soc_balance_constraint(b, t):
            if t == model.time.first():
                return b.soc[t] == b.soc_start*b.capacity + model.dt * (b.pc[t]*b.effc - b.pd[t]*(1/b.effd) - b.psd)
            return b.soc[t] == b.soc[t-1] + model.dt * (b.pc[t]*b.effc - b.pd[t]*(1/b.effd) - b.psd)

        @block.Constraint()
        def soc_end_constraint(b):
            return b.soc[model.time.last()] >= b.soc_start*b.capacity

    def objective_cost(self, block):
        ## Objective cost
        @block.Expression()
        def cost(m):
            return 0.0
    
    def recover_results(self, block):
        model = block.model()
        return {
            "power_dc" : np.array([opt.value(block.power_dc[t]) for t in model.time]),
            "soc"   : np.array([opt.value(block.soc[t]) for t in model.time])
        }

class EnergyReservoirKineticModel(EnergyReservoirModel):
    """EnergyReservoirKineticModel extends the EnergyReservoirModel by differentiating available and bound energy reservoirs"""
    def __init__(self, capacity, power, kinetic_params) -> None:
        super().__init__(capacity=capacity, power=power)
        self._kintetik_params = kinetic_params

    def build(self, block) -> None:
        super().build(block)
        self.kinetic_params(block)
        self.kinetic_constraints(block)

    def kinetic_params(self, block):
        rate_c, rate_d = self._kintetik_params
        block.rate_c = opt.Param(initialize=rate_c)
        block.rate_d = opt.Param(initialize=rate_d)
        # mc: slope     of charge power limit
        # bc: intercept of charge power limit
        @block.Expression()
        def mc(b):
            return -b.max_power / (b.rate_c * b.capacity)
        @block.Expression()
        def bc(b):
            return b.soc_max * (b.max_power / b.rate_c)

        @block.Expression()
        def md(b):
            return -b.max_power / (b.rate_d * b.capacity)
        @block.Expression()
        def bd(b):
            return b.soc_min * (b.max_power / b.rate_d)

    def kinetic_constraints(self, block):
        model = block.model()

        @block.Constraint(model.time)
        def kinetic_charge_constraint(b, t):
            return b.pc[t] - b.pd[t] <= b.mc*b.soc[t] + b.bc

        @block.Constraint(model.time)
        def kinetic_discharge_constraint(b, t):
            return b.pc[t] - b.pd[t] >= b.md*b.soc[t] + b.bd

class EnergyReservoirDimensionModel(EnergyReservoirModel):
    def __init__(self, capacity_cost, power_cost) -> None:
        self._capacity_cost = capacity_cost
        self._power_cost = power_cost

    def build(self, block):
        self.storage_dimension_variables(block)
        self.power_variable(block)
        self.soc_variable(block)

        self.constraint_soc(block)

        self.storage_dimension_cost(block)

    def storage_dimension_variables(self, block):
        # params
        block.capacity_cost = opt.Param(initialize=self._capacity_cost, mutable=True)
        block.power_cost    = opt.Param(initialize=self._power_cost, mutable=True)
        
        # variables
        block.capacity  = opt.Var(within=opt.NonNegativeReals)
        block.max_power = opt.Var(within=opt.NonNegativeReals)

    def power_variable(self, block):
        model = block.model()
        
        block.pc = opt.Var(model.time, within=opt.NonNegativeReals)
        block.pd = opt.Var(model.time, within=opt.NonNegativeReals)

        @block.Constraint(model.time)
        def power_charge_limit_constraint(b, t):
            return b.pc[t] <= b.max_power
        
        @block.Constraint(model.time)
        def power_discharge_limit_constraint(b, t):
            return b.pd[t] <= b.max_power

        @block.Expression(model.time)
        def power_dc(b, t):
            return b.pc[t] - b.pd[t]
        
    def soc_variable(self, block):
        model = block.model()

        block.soc_min = opt.Param(
            within=opt.NonNegativeReals,
            default=0.0,
            mutable=True
        )
        block.soc_max = opt.Param(
            within=opt.NonNegativeReals,
            default=1.0,
            mutable=True
        )
        block.soc_start = opt.Param(
            within=opt.NonNegativeReals,
            default=0.5,
            mutable=True
        )

        block.soc = opt.Var(model.time, within=opt.NonNegativeReals)

        @block.Constraint(model.time)
        def soc_lower_bound_constraint(b, t):
            return b.soc[t] >= b.soc_min * b.capacity

        @block.Constraint(model.time)
        def soc_upper_bound_constraint(b, t):
            return b.soc[t] <= b.soc_max * b.capacity

    def storage_dimension_cost(self, block):
        @block.Expression()
        def cost(b):
            return b.capacity * b.capacity_cost + b.max_power * b.power_cost

    def recover_results(self, block):
        results: dict = super().recover_results(block)
        results["capacity"]  = opt.value(block.capacity)
        results["max_power"] = opt.value(block.max_power)
        return results
