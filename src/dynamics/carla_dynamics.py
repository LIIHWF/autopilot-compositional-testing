from .common import Dynamics, interpolate_1d, interpolate_2d
from .experimental_dynamics import ExperimentalDynamics


class CarlaDynamics(ExperimentalDynamics):
    def __init__(self, vl = 80/3.6):
        b_table = {
            0.0: 0.0,
            5.0: 0.8,
            5.4: 1.3,
            10.0: 6.8,
            12.5: 11.0,
            15.0: 15.8,
            15.3: 16.3,
            16.0: 17.6,
            18.5: 23.1,
            18.9: 24.1,
            20.0: 26.0,
            80/3.6: 31.5
        }

        av_table = {
            (0.0, 0.0): 0.0,
            (0.0, 24.0): 15.3,
            (5.0, 0.8): 5.4,
            (5.0, 24.8): 16.0,
            (10.0, 6.8): 12.5,
            (10.0, 30.8): 18.9,
            (15.0, 15.8): 18.5,
            (15.0, 39.8): 22.2,
            (20.0, 50.0): 22.2,
        }

        at_table = {
            (0.0, 0.0): 0.0,
            (0.0, 24.0): 3.25,
            (5.0, 0.8): 0.25,
            (5.0, 24.8): 2.5,
            (10.0, 6.8): 0.7,
            (10.0, 30.8): 2.2,
            (15.0, 15.8): 1.05,
            (15.0, 39.8): 2.2,
            (20.0, 50.0): 2.35,
        }

        super().__init__(vl, b_table, av_table, at_table)
