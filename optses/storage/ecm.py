import math
import numpy as np
import pyomo.environ as opt

from optses.storage.abstract_storage import AbstractStorageModel


class RintModel(AbstractStorageModel):
    "Internal resistance equivalent circuit model"

    def __init__(
        self,
        capacity: float,
        ocv,  # callable
        r0: float,
        circuit: dict,
        i_bounds: tuple[float, float] = None,
        soc_bounds: tuple[float, float] = (0.0, 1.0),
        soc_start: float = 0.5,
        eff: float = 0.99,
    ) -> None:
        # circuit
        self._parallel = circuit["p"]
        self._serial = circuit["s"]

        # params
        self._capacity = capacity
        self._ocv = ocv
        self._r0 = r0
        self._soc_bounds = soc_bounds
        self._soc_start = soc_start
        self._eff = eff

        if i_bounds is None:
            i_bounds = (capacity, capacity)
        self._i_bounds = i_bounds

    def build(self, block) -> None:
        model = block.model()

        # params
        block.capacity = opt.Param(initialize=self._capacity, mutable=True)  # kWh
        block.cell_capacity = opt.Param(initialize=self._capacity, mutable=True)  # Ah
        # block.power = opt.Param(initialize=self._power, mutable=True)
        block.cell_parallel = opt.Param(initialize=self._parallel)
        block.cell_serial = opt.Param(initialize=self._serial)

        block.soc_min = opt.Param(
            within=opt.NonNegativeReals, initialize=self._soc_bounds[0], mutable=True
        )
        block.soc_max = opt.Param(
            within=opt.NonNegativeReals, initialize=self._soc_bounds[1], mutable=True
        )
        block.soc_start = opt.Param(
            within=opt.NonNegativeReals, initialize=self._soc_start, mutable=True
        )

        block.effc = opt.Param(within=opt.PercentFraction, initialize=self._eff)
        block.effd = opt.Param(within=opt.PercentFraction, initialize=self._eff)

        # block.r0 = opt.Param(default=0.001096)
        block.r0 = opt.Param(initialize=self._r0, mutable=True)

        # vars
        block.ic = opt.Var(
            model.time, within=opt.NonNegativeReals, bounds=(0, self._i_bounds[0])
        )
        block.id = opt.Var(
            model.time, within=opt.NonNegativeReals, bounds=(0, self._i_bounds[1])
        )

        block.ocv = opt.Var(model.time, within=opt.NonNegativeReals)
        block.v = opt.Var(
            model.time, within=opt.NonNegativeReals
        )  # , bounds=(block.vmin, block.vmax)

        block.cell_power = opt.Var(model.time, within=opt.Reals)

        block.soc = opt.Var(model.time, within=opt.UnitInterval)

        # constraints
        @block.Constraint(model.time)
        def soc_constraint(b, t):
            if t == model.time.first():
                return (
                    b.soc[t]
                    == b.soc_start
                    + model.dt
                    * (b.ic[t] * b.effc - b.id[t] * (1 / b.effd))
                    / b.cell_capacity
                )
            return (
                b.soc[t]
                == b.soc[t - 1]
                + model.dt
                * (b.ic[t] * b.effc - b.id[t] * (1 / b.effd))
                / b.cell_capacity
            )

        # @block.Constraint()
        # def soc_end_constraint(b):
        #     return b.soc[model.time.last()] >= b.soc_start

        @block.Constraint(model.time)
        def soc_bounds_lower(b, t):
            return b.soc[t] >= b.soc_min

        @block.Constraint(model.time)
        def soc_bounds_upper(b, t):
            return b.soc[t] <= b.soc_max

        block.ocv_constraint = opt.Constraint(model.time, rule=self._ocv)

        @block.Expression(model.time)
        def i(b, t):
            return b.ic[t] - b.id[t]

        @block.Constraint(model.time)
        def voltage_constraint(b, t):
            return b.v[t] == (b.ocv[t] + b.r0 * b.i[t])

        @block.Constraint(model.time)
        def power_constraint(b, t):
            return b.cell_power[t] == b.v[t] * b.i[t]

        @block.Expression(model.time)
        def power_dc(b, t):
            return b.cell_power[t] * b.cell_parallel * b.cell_serial

        @block.Expression(model.time)
        def i_dc(b, t):
            return b.i[t] * b.cell_parallel

        @block.Expression(model.time)
        def v_dc(b, t):
            return b.v[t] * b.cell_serial

        @block.Expression(model.time)
        def ocv_dc(b, t):
            return b.ocv[t] * b.cell_serial

        @block.Expression(model.time)
        def cell_loss(b, t):
            return b.r0 * (b.ic[t] + b.id[t]) ** 2

        @block.Expression(model.time)
        def cell_loss_dc(b, t):
            return b.cell_loss[t] * b.cell_parallel * b.cell_serial

        # self.degradation_model(block)
        # @block.Expression()
        # def cost(b):
        #     return b.degradation_cost

    def degradation_model(self, block) -> None:
        model = block.model()

        # Calendaric degradation (with constant temperature)
        block.ksoc_ref = opt.Param(initialize=2.857)
        block.ksoc_const = opt.Param(initialize=0.60225)
        block.k_T = opt.Param(initialize=1.2571e-5)  # ~ 25°C
        block.soh = opt.Param(initialize=0.99, mutable=True)

        @block.Expression(model.time)
        def k_soc(b, t):
            return b.ksoc_ref * (b.soc[t] - 0.5) ** 3 + b.ksoc_const

        @block.Expression()
        def calendaric_degradation(b):
            return (
                sum(((b.k_soc[t] * b.k_T) ** 2) / (2 * (1 - b.soh)) for t in model.time)
                * model.dt
                * 3600
            )

        # @block.Expression()
        # def k_soc(b):
        #     return sum(b.ksoc_ref * (b.soc[t] - 0.5)**3 + b.ksoc_const for t in model.time) / 96 # mean soc

        # @block.Expression()
        # def calendaric_degradation(b):
        #     return ((b.k_soc * b.k_T) ** 2) / (2 * (1 - b.soh)) * 96 * model.dt

        # Cyclic degradation
        block.soc_min_dod = opt.Var(within=opt.UnitInterval)
        block.soc_max_dod = opt.Var(within=opt.UnitInterval)
        block.kdod_ref = opt.Param(initialize=4.0253)
        block.kdod_const = opt.Param(initialize=1.0923)
        block.kcrate_ref = opt.Param(initialize=0.0630)
        block.kcrate_const = opt.Param(initialize=0.0971)

        @block.Expression()
        def fec(b):
            return (
                model.dt
                * sum(b.ic[t] + b.id[t] for t in model.time)
                / (2 * b.cell_capacity)
            )

        @block.Constraint(model.time)
        def soc_min_constraint(b, t):
            return b.soc[t] >= b.soc_min_dod

        @block.Constraint(model.time)
        def soc_max_constraint(b, t):
            return b.soc[t] <= b.soc_max_dod

        @block.Expression()
        def dod(b):
            return b.soc_max_dod - b.soc_min_dod

        @block.Expression()
        def k_dod(b):
            "DOD stress factor"
            return b.kdod_ref * (b.dod - 0.6) ** 3 + b.kdod_const

        @block.Expression()
        def crate(b):
            "Average c-rate"
            T = len(model.time) * model.dt  # horizon length in h
            return b.fec * 2 / T

        @block.Expression()
        def k_crate(b):
            return b.kcrate_ref * b.crate + b.kcrate_const

        @block.Expression()
        def cyclic_degradation(b):
            return (
                ((b.k_dod * b.k_crate) ** 2) * b.fec / (2 * 100 * (1 - b.soh)) / 100
            )  # p.u. -> % (100)

        ##
        block.storage_cost = opt.Param(initialize=1, mutable=True)  # $/Wh
        # block.storage_cost_factor = opt.Param(initialize=1, mutable=True) #

        # objective
        block.initial_capacity = opt.Param(
            within=opt.NonNegativeReals, initialize=opt.value(block.capacity)
        )
        block.eol = opt.Param(within=opt.UnitInterval, initialize=0.8, mutable=True)

        @block.Expression()
        def degradation_cost(b):
            return (
                (b.calendaric_degradation + b.cyclic_degradation)
                * b.storage_cost
                * b.initial_capacity
                / (1 - b.eol)
            )  # * b.storage_cost_factor

    def recover_results(self, block) -> dict:
        model = block.model()
        return dict(
            power_dc=np.array([opt.value(block.power_dc[t]) for t in model.time]),
            soc=np.array([opt.value(block.soc[t]) for t in model.time]),
            voltage=np.array([opt.value(block.v[t]) for t in model.time]),
            current=np.array([opt.value(block.i[t]) for t in model.time]),
        )
