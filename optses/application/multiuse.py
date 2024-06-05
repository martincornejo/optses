import numpy as np
import pandas as pd
import pyomo.environ as opt

from pyomo.environ import Model
from optses.application.abstract_application import AbstractApplication


class MultiUse(AbstractApplication):
    def __init__(self, *applications) -> None:
        self._applications = applications

    def build(self, block: Model) -> None:
        for app in self._applications:
            block.add_component(app.name, opt.Block(rule=app.build))

        @block.Expression()
        def cost(b):
            return sum(block.find_component(app.name).cost for app in self._applications)
