import pandas as pd
import argparse
import os
import json
import matplotlib.pyplot as plt
from matplotlib import rcParams
from script.run_test_case.dashboard import Dashboard
import matplotlib
import time

parser = argparse.ArgumentParser('Show test results')
parser.add_argument('autopilot', type=str, help='autopilot to test', choices=['apollo', 'autoware', 'carla', 'lgsvl'])
parser.add_argument('vista_type', type=str, help='type of vista', choices=['merging', 'lane_change', 'crossing_with_yield_signs', 'crossing_with_traffic_lights'])
parser.add_argument('-ve', required=True, type=float, help='speed of the ego vehicle')

args = parser.parse_args()

Dashboard(args.autopilot, args.vista_type, args.ve).show(True)
