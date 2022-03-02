import numpy as np
import pyomo.environ as opt

from optses.storage.abstract_storage import AbstractStorageModel


class ChargeReservoirModel(AbstractStorageModel):
    def __init__(
        self,
        capacity: float,
        cell_capacity: float = 0.04,
        imax: float = None,
        voc_params: tuple = None,
        r0: float = None,
        soc_bounds: tuple[float, float] = (0.0, 1.0),
        soc_start: float = 0.5,
        effc: float = 0.97,
        effd: float = None,
        csd: float = 0.0,
    ) -> None:
        if voc_params is None:
            voc_params = (0.962857, -0.717143, 0.41, 3.445)

        if r0 is None:
            r0 = 0.001096

        if effd is None:
            effd = effc

        self._capacity = capacity

        # TODO: properly scale ECM to storage size
        self._ncells = capacity / cell_capacity  # number of cells (4 Wh cell capacity)
        self.imax = imax

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
        block.capacity = opt.Param(initialize=self._capacity, mutable=True)
        # block.power = opt.Param(initialize=self._power, mutable=True)
        block.n_cells = opt.Param(initialize=self._ncells)

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
        voc_a, voc_b, voc_c, voc_d = self._voc_params
        block.voc_a = opt.Param(initialize=voc_a)
        block.voc_b = opt.Param(initialize=voc_b)
        block.voc_c = opt.Param(initialize=voc_c)
        block.voc_d = opt.Param(initialize=voc_d)

        block.r0 = opt.Param(default=0.001096)

        # vars
        block.ic = opt.Var(
            model.time, within=opt.NonNegativeReals, bounds=(0, self.imax)
        )
        block.id = opt.Var(
            model.time, within=opt.NonNegativeReals, bounds=(0, self.imax)
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
                    / b.capacity
                )
            return (
                b.soc[t]
                == b.soc[t - 1]
                + model.dt
                * (b.ic[t] * b.effc - b.id[t] * (1 / b.effd) - b.csd)
                / b.capacity
            )  # ?

        @block.Constraint()
        def soc_end_constraint(b):
            return b.soc[model.time.last()] >= b.soc_start

        @block.Constraint(model.time)
        def voc_constraint(b, t):
            return (
                b.voc[t]
                == b.voc_a * b.soc[t] ** 3
                + b.voc_b * b.soc[t] ** 2
                + b.voc_c * b.soc[t]
                + b.voc_d
            )

        @block.Constraint(model.time)
        def voltage_constraint(b, t):
            return b.v[t] == (b.voc[t] + b.r0 * (b.ic[t] - b.id[t])) * b.n_cells

        @block.Constraint(model.time)
        def power_constraint(b, t):
            return b.power_dc[t] == b.v[t] * (b.ic[t] - b.id[t]) / 1000  # W in kW...

        @block.Expression(model.time)
        def i(b, t):
            return b.ic[t] - b.id[t]

        # objective
        @block.Expression()
        def cost(b):
            return 0.0

    def recover_results(self, block) -> dict:
        model = block.model()
        return dict(
            power_dc=np.array([opt.value(block.power_dc[t]) for t in model.time]),
            soc=np.array([opt.value(block.soc[t]) for t in model.time]),
            voltage=np.array([opt.value(block.v[t]) for t in model.time]),
            current=np.array([opt.value(block.i[t]) for t in model.time]),
        )
