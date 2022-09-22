import numpy as np
import pandas as pd
import pyomo.environ as opt

from pyomo.environ import Model
from optses.application.abstract_application import AbstractApplication


class MultiUse(AbstractApplication):
    def __init__(self, *applications) -> None:
        self._applications = applications

    @property
    def profile(self):
        return self._applications[0].profile # 

    def build(self, model: Model) -> None:
        for application_model in self._applications:
            model.add_component(
                application_model.name,
                opt.Block(rule=application_model.build)
            )