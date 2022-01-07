import pyomo.environ as opt

from optses.storage.abstract_storage import AbstractStorageModel


class ChargeReservoirModel(AbstractStorageModel):
    def __init__(self, capacity, power) -> None:
        self._capacity  = capacity
        self._power_max = power
        self._ncells # number of cells

        self._soc_bounds

        self._effc
        self._effd
    
    def build(self, block) -> None:
        model = block.model()     

        # params
        block.capacity = opt.Param(initialize=self._capacity, mutable=True)
        block.power = opt.Param(initialize=self._power, mutable=True)
        block.n_cells = opt.Param(initialize=self._ncells)

        block.soc_min = opt.Param()
        block.soc_max = opt.Param()
        block.soc_start = opt.Param()

        block.effc = opt.Param()
        block.effd = opt.Param()
        block.sd   = opt.Param()

        block.imin = opt.Param()
        block.imax = opt.Param()

        block.vmin = opt.Param()
        block.vmax = opt.Param()

        block.voc_a = opt.Param()
        block.voc_b = opt.Param()
        block.voc_c = opt.Param()
        block.voc_d = opt.Param()

        block.r0 = opt.Param()
        # block.n_cells = opt.Param()

        # vars
        block.ic = opt.Var(model.time, within=opt.NonNegativeReals)
        block.id = opt.Var(model.time, within=opt.NonNegativeReals)
        
        block.voc = opt.Var(model.time, within=opt.NonNegativeReals)
        block.v   = opt.Var(model.time, within=opt.NonNegativeReals)
        
        block.power_dc = opt.Var(model.time, within=opt.Reals)

        block.soc = opt.Var(model.time, within=opt.NonNegativeReals)

        # constraints
        @block.Constraint(model.time)
        def soc_constraint(b, t):
            if t == model.time.first():
                return b.soc[t] == b.soc_start + model.dt * (b.ic[t] * b.effc - b.id[t] * (1/b.effd) - b.psd)
            return b.soc[t] == b.soc[t-1] + model.dt * (b.ic[t] * b.effc - b.id[t] * (1/b.effd) - b.sd) # / capacity ?

        @block.Constraint()
        def soc_end_constraint(b):
            return b.soc[model.time.last()] >= b.soc_start

        @block.Constraint(model.time)
        def voc_constraint(b, t):
            return b.voc[t] == b.voc_a * b.soc[t]**3 + b.voc_b * b.soc[t]**2 + b.voc_c * b.soc[t] + b.voc_d

        @block.Constraint(model.time)
        def voltage_constraint(b, t):
            return b.v[t] == (b.voc[t] + b.r0*(b.ic[t] - b.id[t])) # * n_cells

        @block.Constraint(model.time)
        def power_constraint(b, t):
            return b.power_dc[t] == b.v[t] * (b.ic[t] - b.id[t]) # / 1000 W in kW...

