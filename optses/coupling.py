class Load:
    def __init__(self, profile):
        self.profile = profile

    def build(self, block):
        model = block.model()
        block.power = opt.Param(
            model.time,
            within=opt.Reals,
            mutable=True,
            initialize=lambda b, t: self.profile.iloc[t],
        )


class Grid:
    def __init__(self):
        pass

    def build(self, block):
        model = block.model()
        block.power_buy = opt.Var(model.time, within=opt.NonNegativeReals)
        block.power_sell = opt.Var(model.time, within=opt.NonNegativeReals)

        @block.Expression(model.time)
        def power(b, t):
            return b.power_buy[t] - b.power_sell[t]


class GridSlack:
    def __init__(self, penalty: float):
        self.penalty = penalty

    def build(self, block):
        model = block.model()
        block.slack_penalty = opt.Param(
            within=opt.NonNegativeReals, initialize=self.penalty
        )
        block.power_slack_buy = opt.Var(model.time, within=opt.NonNegativeReals)
        block.power_slack_sell = opt.Var(model.time, within=opt.NonNegativeReals)

        @block.Expression(model.time)
        def power(b, t):
            return b.power_slack_buy[t] - b.power_slack_sell[t]

        @block.Expression()
        def penalty_cost(b):
            return (
                sum((b.power_slack_buy[t] + b.power_slack_sell[t]) for t in model.time)
                * b.slack_penalty
            )