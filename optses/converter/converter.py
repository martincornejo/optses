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

        block.effc = opt.Param(within=opt.PercentFraction, initialize=self._effc)
        block.effd = opt.Param(within=opt.PercentFraction, initialize=self._effd)

        # block.pemax = opt.Param(within=opt.NonNegativeReals, initialize=self._power)

        block.pec = opt.Var(model.time)  # bound method?
        block.ped = opt.Var(model.time)

        @block.Constraint(model.time)
        def converter_efficiency(b, t):
            return b.power_dc[t] == b.effc * b.pec[t] - (1 / b.effd) * b.ped[t]

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
        self._k0 = k0
        self._k1 = k1
        self._k2 = k2

    def build(self, block) -> None:
        block.k0 = opt.Param(within=opt.Reals, initialize=self._k0)
        block.k1 = opt.Param(within=opt.Reals, initialize=self._k1)
        block.k2 = opt.Param(within=opt.Reals, initialize=self._k2)

        block.pemax = opt.Param(within=opt.NonNegativeReals, initialize=self._power)

        block.power = opt.Var(block.time)

        @block.Constraint(block.time)
        def converter_efficiency(b, t):
            return b.power_dc[t] == b.k0 + b.k1 * b.power[t] + b.k2 * b.power[t] ** 2


class RampinelliFitConverter(AbstractConverter):
    def __init__(self, power, k0, k1, k2) -> None:
        self._power = power
        self._k0 = k0
        self._k1 = k1
        self._k2 = k2

    def build(self, block):
        block.k0 = opt.Param(
            within=opt.Reals, initialize=self._k0
        )  # charge and discharge params?
        block.k1 = opt.Param(within=opt.Reals, initialize=self._k1)
        block.k2 = opt.Param(within=opt.Reals, initialize=self._k2)

        block.pemax = opt.Param(within=opt.NonNegativeReals, initialize=self._power)

        # TODO:

        # @block.Constraint(block.time)
        # def converter_efficiency(b, t):

        # pr = pe/pmax
        # eff = pr / (pr + k0 + k1*pr + k2*pr**2)
        # p = pe * eff ?


class NottonFitConverter(RampinelliFitConverter):
    def __init__(self, power, k0, k2) -> None:
        super().__init__(power, k0=k0, k1=0.0, k2=k2)

