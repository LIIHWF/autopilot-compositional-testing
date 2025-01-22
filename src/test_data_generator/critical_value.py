import argparse
from src.dynamics import LgsvlDynamics, CarlaDynamics, ApolloDynamics, AutowareDynamics, Dynamics, LincolnPidDynamics, MileDynamics
import json
import sys


class CriticalComputation:
    def __init__(self, dynamics: Dynamics, vl=80/3.6, dc=24, check_traffic_light_reach = True):
        self.dynamics = dynamics
        self.vl = vl
        self.dc = dc
        self.check_traffic_light_reach = check_traffic_light_reach

    def merging_parameter(self, ve):
        vl = self.vl
        xe = self.dynamics.B(ve)
        av = self.dynamics.AV(ve, xe)
        at = self.dynamics.AT(ve, xe)
        xa = self.dynamics.B(vl) + at * vl
        xf = self.dynamics.B(av)
        
        return {
            've': ve,
            'xe': xe,
            'xa': xa,
            'xf': xf
        }

    def crossing_parameter(self, ve):
        vl = self.vl
        dc = self.dc

        xe = self.dynamics.B(ve)

        av = self.dynamics.AV(ve, xe + dc)
        at = self.dynamics.AT(ve, xe + dc)
        
        xa = at * vl
        xf = self.dynamics.B(av)

        return {
            've': ve,
            'xe': xe,
            'xa': xa,
            'xf': xf
        }
    
    def traffic_light_parameter(self, ve):
        vl = self.vl
        dc = self.dc
        
        xe = self.dynamics.B(ve)
        
        av = self.dynamics.AV(ve, xe + dc)
        at = self.dynamics.AT(ve, xe + dc)

        assert av <= self.vl, f'failed for ve={ve}, got av={av}'

        if self.check_traffic_light_reach:
            ate = self.dynamics.AT(ve, xe)
        else:
            ate = 0

        if at > 5 or ate > 3:
            xf = -1
        else:
            xf = self.dynamics.B(av)
        
        return {
            've': ve,
            'xe': xe,
            'xf': xf
        }
        
    def lane_change_parameter(self, ve):
        vl = self.vl
        p = 13.52  # 3.5 / math.sin(math.radians(15))
        
        xe = self.dynamics.B(ve)
        
        av = ve
        at = p / ve

        xa = self.dynamics.B(vl) + at * vl  # - self.dynamics.B(ve)
        xf = self.dynamics.B(av) 

        return {
            've': ve,
            'xe': xe,
            'xa': xa,
            'xf': xf
        }

    def compute(self, vista, ve):
        if vista == 'merging':
            return self.merging_parameter(ve)
        elif vista == 'crossing':
            return self.crossing_parameter(ve)
        elif vista == 'lane_change':
            return self.lane_change_parameter(ve)
        elif vista == 'traffic_light':
            return self.traffic_light_parameter(ve)
        else:
            raise ValueError(f'Invalid vista {vista}')
        
    def compute_round(self, vista, ve):
        # print(vista, ve)
        return {k: round(v, 1) for k, v in self.compute(vista, ve).items()}


if __name__ == '__main__':
    args = argparse.ArgumentParser()
    args.add_argument('autopilot', choices=['apollo', 'autoware', 'carla', 'lgsvl', 'lincoln_pid', 'mile'], type=str)
    args.add_argument('vista_type', choices=['merging', 'crossing_with_yield_signs', 'lane_change', 'crossing_with_traffic_lights', 'crossing', 'traffic_light'], type=str)
    args.add_argument('ve', type=float)
    args.add_argument('vl', type=float)
    args.add_argument('dc', type=float)

    args = args.parse_args()
    
    if args.vista_type == 'crossing_with_yield_signs':
        args.vista_type = 'crossing'
    
    if args.vista_type == 'crossing_with_traffic_lights':
        args.vista_type = 'traffic_light'

    if args.autopilot == 'lgsvl':
        dynamics = LgsvlDynamics(args.vl)
    elif args.autopilot == 'carla':
        dynamics = CarlaDynamics(args.vl)
    elif args.autopilot == 'apollo':
        dynamics = ApolloDynamics(args.vl)
    elif args.autopilot == 'autoware':
        dynamics = AutowareDynamics(args.vl)
    elif args.autopilot == 'lincoln_pid':
        dynamics = LincolnPidDynamics(args.vl)
    elif args.autopilot == 'mile':
        dynamics = MileDynamics(args.vl)
    else:
        raise ValueError(f'Invalid autopilot {args.autopilot}')
    
    critical_computation = CriticalComputation(dynamics, check_traffic_light_reach=(args.autopilot != 'carla'), vl=args.vl, dc=args.dc)
    
    result = critical_computation.compute(args.vista_type, args.ve)
    print(json.dumps(result))
