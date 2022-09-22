from pyomo.core.base.param import ScalarParam, IndexedParam
from optses.application.abstract_application import AbstractApplication
from optses.storage_system import AbstractStorageSystem

# from optses.timeseries import merge_profiles, get_time_steps, get_time_delta

import pyomo.environ as opt
import numpy as np
import pandas as pd
# class OptModelConfig:


class OptModel:
    def __init__(self, storage_system: AbstractStorageSystem, application: AbstractApplication, power_profile: pd.Series = None, price_profile: pd.DataFrame = None) -> None:
        # TODO: relax input "typing", check if input is not list (or iterable) and convert it to one
        self.model = opt.ConcreteModel()
        self._storage = storage_system
        self._application = application
        self._power_profile = power_profile
        self._price_profile = price_profile

        self._build_model()

    # def update_model(self, data):
    #     self._storage_model.update_model(self.model, data.storage)
    #     # m = self._application_model.update_model(m, data.application)

    # TODO: make glue/constructor methods private, build_model/update_model should provide the full interface

    def _build_model(self) -> None:
        model = self.model
        
        # time parameters
        self.add_time_parameters(model)

        # profiles
        self.add_profiles(model)

        # grid power
        self.add_grid_power_variables(model)
        
        # storage system
        model.add_component(self._storage.name, opt.Block(rule=self._storage.build))

        # application
        model.add_component(self._application.name, opt.Block(rule=self._application.build))

        # power balance constraints
        self.binding_constraints(model)

        ## objective
        self.objective(model)

    def update_model(self, val_dict: dict) -> None:
        model = self.model

        for block_name, block_values in val_dict.items():
            block = model.find_component(block_name)
            for param_name, val in block_values.items():
                param = block.find_component(param_name)
                if isinstance(param, ScalarParam):
                    param.set_value(val)
                elif isinstance(param, IndexedParam):
                    for t in model.time:
                        param[t].set_value(val.iloc[t])

    def add_time_parameters(self, model, config=None):
        # profile = merge_profiles(self._application_model) # TODO: get profile when multiple applications?
        if config is None:
            profile = self._power_profile
            timesteps = len(profile)
            timedelta = profile.index[1] - profile.index[0]
            dt        = timedelta.seconds / 3600
        
        model.time = opt.RangeSet(0, timesteps-1)
        model.dt   = opt.Param(initialize=dt)

    def add_profiles(self, model):
        # power profile (load profile)
        if (load := self._power_profile) is not None:
            model.load_power = opt.Param(model.time, within=opt.Reals, initialize=lambda m, t: load.iloc[t])

        # price profile 
        # TODO split sell/buy prices 
        if (price := self._price_profile) is not None:
            model.price = opt.Param(model.time, within=opt.Reals, initialize=lambda m, t: price.iloc[t])

    def add_grid_power_variables(self, model):
        model.grid = opt.Var(model.time, within=opt.NonNegativeReals)
        model.feedin = opt.Var(model.time, within=opt.NonNegativeReals)

        @model.Expression(model.time)
        def grid_power(b, t):
            return b.grid[t] - b.feedin[t]

    def binding_constraints(self, model):
        if self._power_profile is not None:
            @model.Constraint(model.time)
            def power_balance(m, t):
                storage = m.find_component(self._storage.name)
                return m.grid_power[t] - storage.power[t] == m.load_power[t]
        else: # no load
            @model.Constraint(model.time)
            def power_balance(m, t):
                storage = m.find_component(self._storage.name)
                return m.grid_power[t] - storage.power[t] == 0.0

    def objective(self, model):
        @model.Objective()
        def objective(m):
            storage = model.find_component(self._storage.name)
            application = model.find_component(self._application.name)
            return storage.cost + application.cost
            + sum(model.find_component(system.name).cost for system in self._storage_system_models) \
            + sum(model.find_component(application.name).cost for application in self._application_models)

    def recover_results(self):
        model = self.model
        results = dict()

        results["grid"] = np.array([opt.value(model.grid_power[t]) for t in model.time])

        system = self._storage
        name = system.name
        block = model.find_component(name)
        results[name] = system.recover_results(block) 

        application = self._application
        name = application.name
        block = model.find_component(name)
        results[name] = application.recover_results(block)

        # for system in self._storage_system_models:
        #     name = self._system.name
        #     block = model.find_component(name)
        #     results[name] = system.recover_results(block)

        # for application in self._application_models:
        #     name = application.name
        #     block = model.find_component(name)
        #     results[name] = application.recover_results(block)
        
        return results