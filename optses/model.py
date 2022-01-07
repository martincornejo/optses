from optses.application.abstract_application import AbstractApplication
from optses.storage_system import StorageSystem

# from optses.timeseries import merge_profiles, get_time_steps, get_time_delta

import pyomo.environ as opt

# class OptModelConfig:


class OptModel:
    def __init__(self, storage_system: list[StorageSystem], application: AbstractApplication) -> None:
        # TODO: relax input "typing", check if input is not list (or iterable) and convert it to one
        self.model = opt.ConcreteModel()
        self._storage_system_models = storage_system
        self._application_model = application

        self._build_model()

    # def update_model(self, data):
    #     self._storage_model.update_model(self.model, data.storage)
    #     # m = self._application_model.update_model(m, data.application)

    # TODO: make glue/constructor methods private, build_model/update_model should provide the full interface

    def _build_model(self) -> None:
        model = self.model
        
        # time parameters
        self.time_parameters(model)
        
        # storage systems
        for storage in self._storage_system_models:
            model.add_component(storage.name, opt.Block(rule=storage.build))

        # application
        model.add_component(
            self._application_model.name,
            opt.Block(rule=self._application_model.build)
        )

        # power balance constraints
        self.binding_constraints(model)

        ## objective
        self.objective(model)

    def update_model(self, val_dict: dict) -> None:
        model = self.model

        for name, values in val_dict.items():
            component = model.find_component(name)
            for var, val in values.items():
                component.find_component(var).set_value(val)

    def time_parameters(self, model, config=None):
        # profile = merge_profiles(self._application_model) # TODO: get profile when multiple applications?
        if config is None:
            profile = self._application_model.profile
            timesteps = len(profile)
            timedelta = profile.index[1] - profile.index[0]
            dt        = timedelta.seconds / 3600
        
        model.time = opt.RangeSet(0, timesteps-1)
        model.dt   = opt.Param(initialize=dt)



    def binding_constraints(self, model):
        @model.Constraint(model.time)
        def power_balance(m, t):
            # return m.grid[t] - m.feedin[t] \
            return m.find_component(self._application_model.name).power[t] \
                - sum(m.find_component(system.name).power[t] for system in self._storage_system_models) \
                == m.find_component(self._application_model.name).power_profile[t]

    def objective(self, model):
        @model.Objective()
        def objective(m):
            return \
            + sum(model.find_component(system.name).cost for system in self._storage_system_models) \
            + model.find_component(self._application_model.name).cost

    def recover_results(self):
        model = self.model
        results = dict()

        for system in self._storage_system_models:
            name = system.name
            block = model.find_component(name)
            results[name] = system.recover_results(block)

        application = self._application_model
        name = application.name
        block = model.find_component(name)
        results[name] = application.recover_results(block)
        

        return results