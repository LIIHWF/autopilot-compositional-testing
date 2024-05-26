import subprocess
import json
import argparse
import loguru
import os
import time
from script.run_test_case.common import PathManager
from script.run_test_case.dashboard import Dashboard


parser = argparse.ArgumentParser('Run a batch of test cases')
parser.add_argument('autopilot', type=str, help='autopilot to test', choices=['apollo', 'autoware', 'carla', 'lgsvl'])
parser.add_argument('vista_type', type=str, help='type of vista', choices=['merging', 'lane_change', 'crossing_with_yield_signs', 'crossing_with_traffic_lights'])
parser.add_argument('-ve', required=True, type=float, help='speed of the ego vehicle')
parser.add_argument('-min_xa', type=float, help='minimum xa to be tested')
parser.add_argument('-max_xa', type=float, help='maximum xa to be tested')
parser.add_argument('-min_xf', type=float, help='minimum xf to be tested')
parser.add_argument('-max_xf', type=float, help='maximum xf to be tested')
parser.add_argument('--auto-close', action='store_true', help='close the dashboard after all test cases are done', default=False)

args = parser.parse_args()

path_manager = PathManager(args.autopilot, args.vista_type, args.ve)
dashboard = Dashboard(args.autopilot, args.vista_type, args.ve)
dashboard.show(False)


subprocess_obj = None

def on_close(event):
    exit()
    # os._exit(0)

dashboard.fig.canvas.mpl_connect('close_event', on_close)

def run_for(test_case):
    if os.path.exists(path_manager.verdict_path(test_case)):
        loguru.logger.success(f'({args.autopilot}, {args.vista_type}): Test result for {test_case} already exists. Skipping...')
        return False
    
    loguru.logger.info(f'({args.autopilot}, {args.vista_type}): Running test case: {test_case}')
    dashboard.mark_running(test_case)
    
    command: str = f'timeout 300 bazel-bin/script/run_test_case/run_single {args.autopilot} {args.vista_type} -ve {test_case["ve"]} -xf {test_case["xf"]} --json'
    if 'xa' in test_case:
        command += f' -xa {test_case["xa"]}'

    global subprocess_obj
    subprocess_obj = subprocess.Popen(command.split(), shell=False, stdout=subprocess.PIPE)

    while subprocess_obj.poll() is None:
        # dashboard.show(False)
        # dashboard.fig.canvas.draw_idle()
        dashboard.fig.canvas.start_event_loop(1)
        # dashboard.pause(1)

    stdout, stderr = subprocess_obj.communicate()
    try:
        result = json.loads(stdout)
        os.makedirs(path_manager.verdict_dir(), exist_ok=True)
        with open(path_manager.verdict_path(test_case), 'w') as f:
            json.dump(result, f, indent=4)
        return True
    
    except json.JSONDecodeError:
        loguru.logger.warning(f'Failed to run test case: {test_case}. Retrying...')   
        return run_for(test_case)


with open(path_manager.test_case_path()) as f:
    test_cases = json.load(f)

updated = False
for test_case in test_cases:
    if abs(test_case['ve'] - args.ve) < 1e-5:
        if args.min_xa is not None and test_case['xa'] < args.min_xa:
            continue
        if args.max_xa is not None and test_case['xa'] > args.max_xa:
            continue
        if args.min_xf is not None and test_case['xf'] < args.min_xf:
            continue
        if args.max_xf is not None and test_case['xf'] > args.max_xf:
            continue
        # dashboard.show(False)
        if run_for(test_case):
            updated = True
            result = dashboard.query_result(test_case)
            dashboard.update_result(result)

if not args.auto_close:
    dashboard.show(True)