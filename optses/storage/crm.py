import math
import numpy as np
import pyomo.environ as opt

from optses.storage.abstract_storage import AbstractStorageModel


class ChargeReservoirModel(AbstractStorageModel):
    def __init__(
        self,
        capacity: float,
        voltage: float,
        imax: float = None,
        voc_params: tuple = None,
        r0: float = None,
        soc_bounds: tuple[float, float] = (0.0, 1.0),
        soc_start: float = 0.5,
        effc: float = 0.99,
        effd: float = None,
        csd: float = 0.0,
    ) -> None:
        if voc_params is None:
            voc_params = (0.962857, -0.717143, 0.41, 3.445)

        if r0 is None:
            # r0 = 0.001096
            r0 = 0.045

        if effd is None:
            effd = effc

        self._capacity = capacity

        self._cell_capacity = cell_nominal_capacity = 3.0  # Ah
        self._cell_voltage = cell_nominal_voltage = 3.2  # V
        cell_icmax = cell_nominal_capacity * 1.0  # 1.0 C
        cell_idmax = cell_nominal_capacity * 6.6  # 6.6 C

        # TODO: exact size?
        self._parallel = voltage / cell_nominal_voltage  # parallel cells
        self._serial = capacity / voltage / cell_nominal_capacity  # serial cells
        self.icmax = cell_icmax  # * self._serial
        self.idmax = cell_idmax  # * self._serial

        self._voc_params = voc_params
        self._r0 = r0
        self._soc_bounds = soc_bounds
        self._soc_start = soc_start
        self._effc = effc  # charge efficiency
        self._effd = effd  # discharge efficiency
        self._csd = csd  # self-discharge current

    def build(self, block) -> None:
        model = block.model()

        # params
        block.capacity = opt.Param(initialize=self._capacity, mutable=True)  # kWh
        block.cell_capacity = opt.Param(
            initialize=self._cell_capacity, mutable=True
        )  # Ah
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

        block.effc = opt.Param(within=opt.PercentFraction, initialize=self._effc)
        block.effd = opt.Param(within=opt.PercentFraction, initialize=self._effd)
        block.csd = opt.Param(within=opt.NonNegativeReals, initialize=self._csd)

        # block.imin = opt.Param()
        # block.imax = opt.Param(initialize=self.imax)

        # block.vmin = opt.Param()
        # block.vmax = opt.Param()
        # voc_a, voc_b, voc_c, voc_d = self._voc_params
        # block.voc_a = opt.Param(initialize=voc_a)
        # block.voc_b = opt.Param(initialize=voc_b)
        # block.voc_c = opt.Param(initialize=voc_c)
        # block.voc_d = opt.Param(initialize=voc_d)

        block.voc_a = opt.Param(initialize=2.900074932012732)
        block.voc_b = opt.Param(initialize=4.927554283419728)
        block.voc_c = opt.Param(initialize=-29.299690476948975)
        block.voc_d = opt.Param(initialize=95.9761469315301)
        block.voc_e = opt.Param(initialize=-181.93721324798557)
        block.voc_f = opt.Param(initialize=199.7074219198455)
        block.voc_g = opt.Param(initialize=-117.75695789380974)
        block.voc_h = opt.Param(initialize=28.844629090305514)

        # block.r0 = opt.Param(default=0.001096)
        block.r0 = opt.Param(initialize=self._r0)

        # vars
        block.ic = opt.Var(
            model.time, within=opt.NonNegativeReals, bounds=(0, self.icmax)
        )
        block.id = opt.Var(
            model.time, within=opt.NonNegativeReals, bounds=(0, self.idmax)
        )

        block.voc = opt.Var(model.time, within=opt.NonNegativeReals)
        block.v = opt.Var(model.time, within=opt.NonNegativeReals)

        block.power_dc = opt.Var(model.time, within=opt.Reals)

        block.soc = opt.Var(model.time, within=opt.UnitInterval)

        # constraints
        @block.Constraint(model.time)
        def soc_constraint(b, t):
            if t == model.time.first():
                return (
                    b.soc[t]
                    == b.soc_start
                    + model.dt
                    * (b.ic[t] * b.effc - b.id[t] * (1 / b.effd) - b.csd)
                    / b.cell_capacity
                )
            return (
                b.soc[t]
                == b.soc[t - 1]
                + model.dt
                * (b.ic[t] * b.effc - b.id[t] * (1 / b.effd) - b.csd)
                / b.cell_capacity
            )  # ?

        @block.Constraint()
        def soc_end_constraint(b):
            return b.soc[model.time.last()] >= b.soc_start

        @block.Constraint(model.time)
        def soc_bounds_lower(b, t):
            return b.soc[t] >= b.soc_min

        @block.Constraint(model.time)
        def soc_bounds_upper(b, t):
            return b.soc[t] <= b.soc_max

        # optimize?
        @block.Constraint(model.time)
        def voc_constraint(b, t):
            return (
                b.voc[t]
                == b.voc_a
                + b.voc_b * b.soc[t]
                + b.voc_c * b.soc[t] ** 2
                + b.voc_d * b.soc[t] ** 3
                + b.voc_e * b.soc[t] ** 4
                + b.voc_f * b.soc[t] ** 5
                + b.voc_g * b.soc[t] ** 6
                + b.voc_h * b.soc[t] ** 7
            )

        @block.Constraint(model.time)
        def voltage_constraint(b, t):
            return b.v[t] == (b.voc[t] + b.r0 * (b.ic[t] - b.id[t]))

        @block.Expression(model.time)
        def i(b, t):
            return b.ic[t] - b.id[t]

        @block.Constraint(model.time)
        def power_constraint(b, t):
            return (
                b.power_dc[t]
                == b.v[t] * (b.ic[t] - b.id[t]) * b.cell_serial * b.cell_parallel
            )  # W in kW...

        # Calendaric degradation (with constant temperature)
        block.ksoc_ref = opt.Param(initialize=2.857)
        block.ksoc_const = opt.Param(initialize=0.60225)
        block.k_T = opt.Param(initialize=1.2571e-5) # ~ 25°C
        block.soh = opt.Param(initialize=0.99, mutable=True)

        @block.Expression(model.time)
        def k_soc(b, t):
            return b.ksoc_ref * (b.soc[t] - 0.5)**3 + b.ksoc_const

        @block.Expression()
        def calendaric_degradation(b):
            return sum(((b.k_soc[t] * b.k_T) ** 2) / (2 * (1 - b.soh)) for t in model.time) * model.dt

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
            return model.dt * sum(b.ic[t] + b.id[t] for t in model.time) / b.cell_capacity # TODO: scale to Ah
        
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
            return b.kdod_ref * (b.dod - 0.6)**3 + b.kdod_const

        @block.Expression()
        def crate(b):
            "Average c-rate"
            return b.fec / 24 # ?

        @block.Expression()
        def k_crate(b):
            return b.kcrate_ref * b.crate + b.kcrate_const


        @block.Expression()
        def cyclic_degradation(b):
            return ((b.k_dod * b.k_crate)** 2) / (2 * (1 - b.soh)) * b.fec

        ##
        block.storage_cost = opt.Param(initialize=0.3, mutable=True) # $/Wh
        block.storage_cost_factor = opt.Param(initialize=1, mutable=True) #

        # objective
        @block.Expression()
        def cost(b):
            return (b.calendaric_degradation + b.cyclic_degradation) * b.capacity * b.storage_cost * b.storage_cost_factor

    def recover_results(self, block) -> dict:
        model = block.model()
        return dict(
            power_dc=np.array([opt.value(block.power_dc[t]) for t in model.time]),
            soc=np.array([opt.value(block.soc[t]) for t in model.time]),
            voltage=np.array([opt.value(block.v[t]) for t in model.time]),
            current=np.array([opt.value(block.i[t]) for t in model.time]),
        )
 