"""utilities for profile analysis"""
import pyomo.environ as opt

# def merge_profiles(applications):
#     profiles = [p for p in applications.profile]    # error if profiles are not consistent
#     return pd.concat(*profiles, axis=1)

def get_time_steps(profile):
    return len(profile) # pd size instead?

def get_time_delta(profile):
    "Returns dt in hours"
    dt = profile.index[1] - profile.index[0]
    return dt.seconds / 3600

class Profile:
    def __init__(self, load_profile, generation_profile=None) -> None:
        if generation_profile is None:
            generation_profile = load_profile.copy()
            generation_profile[:] = 0.0

        self.profile = load_profile - generation_profile

    def build_model(self, model):
        model.profile = opt.Param(model.time, within=opt.Reals, initialize=lambda m, t: self.profile.iloc[t])
    
    def get_time_steps(self):
        return len(self.profile)

    def get_time_delta(self):
        "Returns dt in hours"
        dt = self.profile.index[1] - self.profile.index[0]
        return dt.seconds / 3600
