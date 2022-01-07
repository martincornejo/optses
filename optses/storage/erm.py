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
        model = block.model()
        
        ## Params
        block.capacity = opt.Param(
            # doc="Energy capacity of the storage",
            within=opt.NonNegativeReals,
            initialize=self._capacity,
            mutable=True
        )
        block.max_power = opt.Param(
            within=opt.NonNegativeReals,
            initialize=self._power,
            mutable=True
        )
        block.soc_min = opt.Param(
            # within=opt.PercentFraction,
            within=opt.NonNegativeReals,
            # validate=lambda m, v: v < m.soc_max, # ?
            default=0.0,
            mutable=True
        )
        block.soc_max = opt.Param(
            # within=opt.PercentFraction,
            within=opt.NonNegativeReals,
            # validate=lambda m, v: v > m.soc_min,
            default=1.0,
            mutable=True
        )
        block.soc_start = opt.Param(
            # within=opt.PercentFraction,
            within=opt.NonNegativeReals,
            # validate=lambda m, v: m.soc_min < v < m.soc_max,
            default=0.5,
            mutable=True
        )
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
        
        ## Variables
        block.pc = opt.Var(model.time, bounds=(0, block.max_power))
        block.pd = opt.Var(model.time, bounds=(0, block.max_power))

        block.soc = opt.Var(model.time, bounds=(block.soc_min*block.capacity, block.soc_max*block.capacity))

        @block.Expression(model.time)
        def power_dc(b, t):
            return b.pc[t] - b.pd[t]

        ## Constraints
        @block.Constraint(model.time)
        def soc_balance_constraint(b, t):
            if t == model.time.first():
                return b.soc[t] == b.soc_start*b.capacity + model.dt * (b.pc[t]*b.effc - b.pd[t]*(1/b.effd) - b.psd)
            return b.soc[t] == b.soc[t-1] + model.dt * (b.pc[t]*b.effc - b.pd[t]*(1/b.effd) - b.psd)

        @block.Constraint()
        def soc_end_constraint(b):
            return b.soc[model.time.last()] >= b.soc_start*b.capacity

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

# class EnergyReservoirKyneticModel(EnergyReservoirModel):
#     def __init__(self, data) -> None:
#         super().__init__(data)

#     def build_model(self, block):
#         super().build(block)
#         self.kinetic_constraints(block)

#     def kinetic_constraints(self):
#         m = self.model

#         m1, m2 = m.m1, m.m2
#         b1, b2 = m.b1, m.b2

#         def kinetic_charge_constraint_rule(m, i):
#             return m.pe_d[i] + m.pe_c[i] <= m2*m.soc[i] + b2
#         m.kinetic_charge_constraint = opt.Constraint(m.time, rule=kinetic_charge_constraint_rule)

#         def kinetic_discharge_constraint_rule(m, i):
#             return m.pe_d[i] + m.pe_c[i] >= m1*m.soc[i] + b1
#         m.kinetic_discharge_constraint = opt.Constraint(m.time, rule=kinetic_discharge_constraint_rule)

# class EnergyReservoirDimensionModel(EnergyReservoirModel):
#     def __init__(self) -> None:
#         super().__init__()

#     def energy_capacity_variable(self):
#         m = self.model
#         m.cap = opt.Var(within=opt.NonNegativeReals)
#         m.pwr = opt.Var(within=opt.NonNegativeReals)

#     def power_limit_constraint(self):
#         pass
