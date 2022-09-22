from abc import ABC, abstractmethod

import numpy as np
import pyomo.environ as opt


class AbstractConverter(ABC):
    @abstractmethod
    def build(self, block) -> None:
        pass

    # @abstractmethod
    # def update_block(self, block):
    #     pass
    def recover_results(self, block):
        model = block.model()
        return {"power": np.array([opt.value(block.power[t]) for t in model.time])}


class IdealConverter(AbstractConverter):
    def build(self, block):
        model = block.model()

        @block.Expression(model.time)
        def power(b, t):
            return b.power_dc[t]


class ConstantEfficiencyConverter(AbstractConverter):
    def __init__(self, effc, effd=None) -> None:
        if effd is None:
            effd = effc

        self._effc = effc
        self._effd = effd

    def build(self, block) -> None:
        model = block.model()

        # TODO: improve naming of variables
        block.c_effc = opt.Param(within=opt.PercentFraction, initialize=self._effc)
        block.c_effd = opt.Param(within=opt.PercentFraction, initialize=self._effd)

        # block.pemax = opt.Param(within=opt.NonNegativeReals, initialize=self._power)

        block.pec = opt.Var(model.time)  # bound method?
        block.ped = opt.Var(model.time)

        @block.Constraint(model.time)
        def converter_efficiency(b, t):
            return b.power_dc[t] == b.c_effc * b.pec[t] - (1 / b.c_effd) * b.ped[t]

        @block.Expression(model.time)
        def power(b, t):
            return b.pec[t] - b.ped[t]


class LinearFitConverter(
    AbstractConverter
):  # NOTE: LinearFit could be a subclass of QuadraticFit
    def __init__(self, power, k0, k1) -> None:
        self._power = power
        self._k0 = k0
        self._k1 = k1

    def build(self, block) -> None:
        block.k0 = opt.Param(within=opt.Reals, initialize=self._k0)
        block.k1 = opt.Param(within=opt.Reals, initialize=self._k1)

        block.pemax = opt.Param(within=opt.NonNegativeReals, initialize=self._power)

        block.power = opt.Var(block.time)

        @block.Constraint(block.time)
        def converter_efficiency(b, t):
            return b.power_dc[t] == b.k1 * b.power[t] + b.k0


class QuadraticFitConverter(AbstractConverter):
    def __init__(self, power, k0, k1, k2) -> None:
        self._power = power
        # TODO: kwarg to specify scaling?
        self._k0 = k0 * power
        self._k1 = k1
        self._k2 = k2 / power

    def build(self, block) -> None:
        model = block.model()

        block.k0 = opt.Param(within=opt.Reals, initialize=self._k0)
        block.k1 = opt.Param(within=opt.Reals, initialize=self._k1)
        block.k2 = opt.Param(within=opt.Reals, initialize=self._k2)

        block.pemax = opt.Param(within=opt.NonNegativeReals, initialize=self._power)

        block.power = opt.Var(model.time)

        @block.Expression(model.time)
        def converter_loss(b, t):
            return b.k0 + b.k1 * b.power[t] + b.k2 * b.power[t] ** 2

        @block.Constraint(model.time)
        def converter_loss_constraint(b, t):
            return b.power_dc[t] <= b.power[t] - b.converter_loss[t]


class RampinelliFitConverter(AbstractConverter):
    def __init__(self, power, k0, k1, k2) -> None:
        self._power = power
        self._k0 = k0
        self._k1 = k1
        self._k2 = k2

    def build(self, block):
        model = block.model()

        # charge and discharge params?
        block.k0 = opt.Param(within=opt.Reals, initialize=self._k0)  
        block.k1 = opt.Param(within=opt.Reals, initialize=self._k1)
        block.k2 = opt.Param(within=opt.Reals, initialize=self._k2)

        block.pemax = opt.Param(within=opt.NonNegativeReals, initialize=self._power)
        block.pec = opt.Var(model.time, within=opt.Reals, bounds=(0, block.pemax))
        block.ped = opt.Var(model.time, within=opt.Reals, bounds=(0, block.pemax))

        @block.Expression(model.time)
        def c_effc(b, t):
            p = b.pec[t] / b.pemax
            return p / (p + b.k0 + b.k1*p + b.k2*p ** 2)

        @block.Expression(model.time)
        def c_effd(b, t):
            p = b.ped[t] / b.pemax
            return p / (p + b.k0 + b.k1*p + b.k2*p ** 2)

        @block.Constraint(model.time)
        def converter_efficiency_constraint(b, t):
            return b.power_dc[t] == b.pec[t] * b.c_effc[t] - b.ped[t] * (1 / b.c_effd[t])

        @block.Expression(model.time)
        def power(b, t):
            return b.pec[t] - b.ped[t]


class NottonFitConverter(RampinelliFitConverter):
    def __init__(self, power, k0, k2) -> None:
        super().__init__(power, k0=k0, k1=0.0, k2=k2)

