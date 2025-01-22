import argparse
import numpy as np
from scipy.interpolate import interp1d

FINELY_TUNED_XE = {
    'apollo-40': {
        'merging': {0.0: 0.0, 2.0: 1.6, 5.0: 6.085806194501845, 10.0: 17.213259316477412, 15.0: 33.1875},
        'lane_change': {5.0: 6.085806194501845, 10.0: 17.213259316477412, 15.0: 31.6875, 20: 50.02083333333333},
        'crossing_with_yield_signs': {0.0: 0, 5.0: 6.085806194501845, 10.0: 17.213259316477412, 15.0: 31.6875},
        'crossing_with_traffic_lights': {0.0: 0, 5.0: 6.085806194501845, 10.0: 17.213259316477412, 15.0: 33.1875, 20.0: 50.02083333333333}
    },
    'apollo': {
        'merging': {0.0: 0.0, 5.0: 6.085806194501845, 10.0: 17.213259316477412, 15.0: 33.1875},
        'lane_change': {5.0: 6.085806194501845, 10.0: 17.213259316477412, 15.0: 31.6875, 20: 50.02083333333333},
        'crossing_with_yield_signs': {0.0: 0, 5.0: 6.085806194501845, 10.0: 17.213259316477412, 15.0: 31.6875},
        'crossing_with_traffic_lights': {0.0: 0, 5.0: 6.085806194501845, 10.0: 17.213259316477412, 15.0: 33.1875, 20.0: 50.02083333333333}
    },
    'autoware': {
        'merging': {0.0: 0, 5.0: 6.2, 10.0: 15.9, 15.0: 33.3},
        'lane_change': {5.0: 6.2, 10.0: 17.3, 15.0: 33.3, 20.0: 54.3},
        'crossing_with_yield_signs': {0.0: 0, 5.0: 6.2, 10.0: 17.3, 15.0: 33.3},
        'crossing_with_traffic_lights': {0.0: 0, 5.0: 6.2, 10.0: 17.3, 15.0: 33.3, 20.0: 58}
    },
    'carla': {
        'merging': {0.0: 0, 5.0: 0.8, 10.0: 6.8, 15.0: 15.8},
        'lane_change': {5.0: 7.0, 10.0: 10.0, 15.0: 15.8, 20.0: 27},
        'crossing_with_yield_signs': {0.0: 0.0, 5.0: 0.8, 10.0: 6.8, 15.0: 15.8},
        'crossing_with_traffic_lights': {0.0: 0.0, 5.0: 1.8, 10.0: 6.8, 15.0: 15.8, 20.0: 26.0}
    },
    'behavior': {
        'merging': {0.0: 0, 5.0: 0.8, 10.0: 6.8, 15.0: 15.8},
        'lane_change': {4.0: 7.0, 5.0: 7.0, 8.0: 10.0, 10.0: 10.0, 15.0: 15.8, 20.0: 27},
        'crossing_with_yield_signs': {0.0: 0.0, 5.0: 0.8, 10.0: 6.8, 15.0: 15.8},
        'crossing_with_traffic_lights': {0.0: 0.0, 5.0: 1.8, 10.0: 6.8, 15.0: 15.8, 20.0: 26.0}
    },
    'lgsvl': {
        'merging': {0.0: 0.0, 5.0: 1.3, 10.0: 2.6, 15.0: 3.8},
        'lane_change': {5.0: 1.3, 10.0: 2.6, 15.0: 3.8, 20.0: 5.1},
        'crossing_with_yield_signs': {0.0: 0.05, 5.0: 1.3, 10.0: 2.6, 15.0: 3.8},
        'crossing_with_traffic_lights': {0.0: 0.05, 5.0: 1.3, 10.0: 2.6, 15.0: 3.8, 20.0: 5.1}
    }
}

def get_finely_tuned_xe(autopilot, vista, ve):
    ve_xe_map = FINELY_TUNED_XE[autopilot][vista]
    for ve_value in ve_xe_map:
        if abs(ve_value - ve) < 0.1:
            return ve_xe_map[ve_value]
    # interpolate using scipy's interp1d for smooth interpolation
    ve_values = list(ve_xe_map.keys())
    ve_values.sort()
    if ve < ve_values[0]:
        raise ValueError(f'Invalid ve value: {ve}, too small')
    if ve > ve_values[-1]:
        raise ValueError(f'Invalid ve value: {ve}, too large')
    # using smooth interpolation
    ve_values = np.array(ve_values)
    xe_values = np.array([ve_xe_map[v] for v in ve_values])
    f = interp1d(ve_values, xe_values, kind='linear', fill_value="extrapolate")
    xe = f(ve)
    return xe

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('autopilot', type=str)
    parser.add_argument('vista', type=str)
    parser.add_argument('ve', type=float)
    args = parser.parse_args()

    xe = get_finely_tuned_xe(args.autopilot, args.vista, args.ve)
    print(xe)