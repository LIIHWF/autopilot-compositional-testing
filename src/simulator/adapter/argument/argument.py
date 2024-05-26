from argparse import ArgumentParser
from src.common.types import *


def make_arg_parser(autopilot: Literal['apollo', 'autoware', 'carla', 'lgsvl'],
                    vista_type: Literal['merging', 'lane_change', 'crossing_with_yield_signs', 'crossing_with_traffic_lights']):
    vista_type_capitalized = ''.join([s.capitalize() for s in vista_type.split('_')])
    autopilot_capitalized = autopilot.capitalize()

    parser = ArgumentParser(
        prog=f'{vista_type_capitalized}Runner for the {autopilot_capitalized} autopilot',
        description=f'Run {vista_type} scenario for the {autopilot_capitalized} autopilot',
    )
    
    parser.add_argument('-ve', required=True, type=float, help='speed of the ego vehicle')
    if vista_type != 'lane_change':
        parser.add_argument('-xe', required=True, type=float, help='distance from the ego vehicle to the critical zone')
    else:
        parser.add_argument('-xff', required=True, type=float, help='distance from the ego vehicle to the front vehicle on the innner lane')
    
    parser.add_argument('-xf', required=True, type=float, help='distance from the critical zone to the front vehicle')
    if vista_type != 'crossing_with_traffic_lights':
        parser.add_argument('-xa', required=True, type=float, help='distance from the arriving vehicle to the critical zone')
    return parser
