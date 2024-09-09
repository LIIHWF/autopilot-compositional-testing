import subprocess
import json
import argparse
import loguru
import os
import time
import pandas as pd
import numpy as np
from typing import Literal


parser = argparse.ArgumentParser('Run a batch of test cases')
parser.add_argument('autopilot', type=str, help='autopilot to test', choices=['apollo', 'autoware', 'carla', 'lgsvl', 'apollo-40', 'behavior'])
parser.add_argument('vista_type', type=str, help='type of vista', choices=['merging', 'lane_change', 'crossing_with_yield_signs', 'crossing_with_traffic_lights'])
parser.add_argument('-ve', required=True, type=float, help='speed of the ego vehicle')
parser.add_argument('-min_xa', type=float, help='minimum xa to be tested', default=0)
parser.add_argument('-max_xa', type=float, help='maximum xa to be tested', default=320)
parser.add_argument('-min_xf', type=float, help='minimum xf to be tested', default = 0)
parser.add_argument('-max_xf', type=float, help='maximum xf to be tested', default = 320)
parser.add_argument('-xa_step', type=float, help='step size for xa', default = 10)
parser.add_argument('-xf_step', type=float, help='step size for xf', default = 40)
parser.add_argument('-xa_min_gap', type=float, help='minimum gap between xa values', default=1)
parser.add_argument('-xf_min_gap', type=float, help='minimum gap between xf values', default=5)
parser.add_argument('--no-run', action='store_true', help='do not run the test cases')
parser.add_argument('--timeout', type=float, help='timeout for each test case', default=None)


args = parser.parse_args()

class PathManager:
    def __init__(self, autopilot, vista, ve):
        self.autopilot = autopilot
        self.vista = vista
        self.ve = ve
        
    def verdict_dir(self):
        save_dir = f'test_result/{self.autopilot}/{self.vista}/ve={int(self.ve + 0.1)}'
        return save_dir

    def verdict_path(self, test_case):
        if self.vista == 'crossing_with_traffic_lights':
            file_name = f'xf={round(test_case["xf"], 1)}.json'
        else:
            file_name = f'xf={round(test_case["xf"], 1)};xa={round(test_case["xa"], 1)}.json'
        verdict = os.path.join(self.verdict_dir(), file_name)
        return verdict
    
    def test_case_path(self):
        path = f'test_case/{self.autopilot}_{self.vista}.json'
        return path


path_manager = PathManager(args.autopilot, args.vista_type, args.ve)


def eq(a, b):
    if a is None or b is None:
        return False
    return abs(a - b) < 1e-3

def query_result(test_case):
    verdict_path = path_manager.verdict_path(test_case)
    if not os.path.exists(verdict_path):
        return None
    with open(verdict_path) as f:
        data = json.load(f)
    for key in test_case:
        if key not in data:
            return None
        if abs(test_case[key] - data[key]) > 1e-5:
            return None
    return data


def is_feasible(data):
    feasible_space = {
        'apollo-40': 19.9,
        'behavior': 6,
        'carla': 31.4,
        'lgsvl': 5.5,
        'autoware': 60.2,
        'apollo': 59.5
    }
    if args.vista_type.startswith('crossing'):
        return True
    else:
        return data['xa'] + data['xf'] > feasible_space[args.autopilot]


start_time = time.time()
total_time = 0.0
simulation_time = 0.0
checking_time = 0.0
generation_time = 0.0
num_of_results = 0


def record_result(result, test_case, refined_xa, refined_xf):
    result_manager.add(result)
    if 'xa' not in test_case:
        if not eq(test_case['xf'], refined_xf) and eq(test_case['xf'], critical_value['xf']):
            grid_result.add(result, True)
        else:
            grid_result.add(result)
    else:
        if ((not eq(test_case['xa'], refined_xa) and eq(test_case['xa'], critical_value['xa']))) or\
            (not eq(test_case['xf'], refined_xf) and eq(test_case['xf'], critical_value['xf'])):
            grid_result.add(result, True)
        else:
            grid_result.add(result)


def run_for(test_case, generation_time=0.0, refined_xa=None, refined_xf=None):
    global total_time, simulation_time, checking_time, num_of_results
    if not is_feasible(test_case):
        loguru.logger.warning(f'({args.autopilot}, {args.vista_type}): Test case {test_case} is not feasible. Skipping...')
        return None
    if os.path.exists(path_manager.verdict_path(test_case)):
        loguru.logger.success(f'({args.autopilot}, {args.vista_type}): Test result for {test_case} already exists. Skipping...')
        result = query_result(test_case)
        if result:
            record_result(result, test_case, refined_xa, refined_xf)
            if args.no_run:
                generation_time = time.time() - start_time
                simulation_time += result['simulation_time']
                checking_time += result['checking_time']
                total_time = generation_time + simulation_time + checking_time
                num_of_results += 1
                
                print(f'Generation time: {generation_time:.2f}s')
                print(f'Simulation time: {simulation_time:.2f}s')
                print(f'Checking time: {checking_time:.2f}s')
                print(f'Total time: {total_time:.2f}s')
                print(f'Number of results: {num_of_results}')
                
                print(result_manager.get_pivot_table())
                
                if args.timeout is not None and total_time > args.timeout:
                    loguru.logger.warning(f'Timeout reached. Exit...')
                    exit()
                
        return None
    if args.no_run:
        loguru.logger.warning(f'({args.autopilot}, {args.vista_type}): Test case {test_case} is not found. Exit...')
        return exit()
    
    loguru.logger.info(f'({args.autopilot}, {args.vista_type}): Running test case: {test_case}')
    
    command: str = f'timeout 90 bazel-bin/script/run_test_case/run_single {args.autopilot} {args.vista_type} -ve {test_case["ve"]} -xf {test_case["xf"]} --json'
    if 'xa' in test_case:
        command += f' -xa {test_case["xa"]}'

    global subprocess_obj
    subprocess_obj = subprocess.Popen(command.split(), shell=False, stdout=subprocess.PIPE)

    stdout, stderr = subprocess_obj.communicate()
    try:
        result = json.loads(stdout)
        result['generation_time'] = generation_time
        os.makedirs(path_manager.verdict_dir(), exist_ok=True)
        with open(path_manager.verdict_path(test_case), 'w') as f:
            json.dump(result, f, indent=4)
        record_result(result, test_case, refined_xa, refined_xf)
        print(result_manager.get_pivot_table())
        return True

    except json.JSONDecodeError:
        loguru.logger.warning(f'Failed to run test case: {test_case}. Retrying...')   
        return run_for(test_case)

CRITICAL_VALUES = {
    'apollo-40': {
        'merging': [{'ve': 0.0, 'xf': 0.0, 'xa': 20.2}, {'ve': 2.0, 'xf': 2.0, 'xa': 28.4}, {'ve': 5.0, 'xf': 7.6, 'xa': 32.5}],
        'lane_change': [{'ve': 5.0, 'xf': 6.1, 'xa': 50.4}, {'ve': 10.0, 'xf': 17.2, 'xa': 35.3}],
        'crossing_with_yield_signs': [{'ve': 0.0, 'xf': 15.4, 'xa': 60.0}, {'ve': 5.0, 'xf': 20.2, 'xa': 42.4}],
        'crossing_with_traffic_lights': [{'ve': 0.0, 'xf': None}, {'ve': 5.0, 'xf': 20.2}, {'ve': 10.0, 'xf': 20.2}]
    },
    'behavior': {
        'merging': [{'ve': 0.0, 'xf': 0.0, 'xa': 5.0}, {'ve': 2.0, 'xf': 2.0, 'xa': 28.4}, {'ve': 5.0, 'xf': 1.2, 'xa': 6.4}],
        'lane_change': [{'ve': 4.0, 'xf': 0.8, 'xa': 35.1}, {'ve': 8.0, 'xf': 6.9, 'xa': 21.0}],
        'crossing_with_yield_signs': [{'ve': 0.0, 'xf': 6.9, 'xa': 32.7}, {'ve': 5.0, 'xf': 6.9, 'xa': 22.74}],
        'crossing_with_traffic_lights': [{'ve': 0.0, 'xf': 6.9}, {'ve': 5.0, 'xf': 6.9}]
    },
}

class ResultManager:
    def __init__(self):
        self.results = []
    
    def add(self, result, only_update=False):
        if result['xa'] is None:
            result = {**result, 'xa': -1}
        for i, history in enumerate(self.results):
            if all(eq(result[key], history[key]) for key in ['ve', 'xa', 'xf']):
                self.results[i] = result
                return
        if not only_update:
            self.results.append(result)
        
    def get_pd_table(self):
        return pd.DataFrame(self.results)
    
    def get_pivot_table(self):
        return self.get_pd_table().pivot(index='xa', columns='xf', values='verdict')
    
    def get_xf_values(self):
        # columns of the pivot table
        return [float(xf) for xf in self.get_pivot_table().columns]
    
    def get_xa_values(self):
        # index of the pivot table
        return [float(xa) for xa in self.get_pivot_table().index]
    
    def get_unknown_data(self):
        # no value at pivot table
        unknown_data = []
        for xa in self.get_xa_values():
            for xf in self.get_xf_values():
                if pd.isna(self.get_pivot_table().loc[xa, xf]):
                    unknown_data.append({'ve': args.ve, 'xa': xa, 'xf': xf})
        return unknown_data
    
    def get_xa_refine_values(self, ignore_gap=False):
        xa_refine_values = set()
        xa_values = self.get_xa_values()
        pivot_table = self.get_pivot_table()
        for i in range(len(xa_values) - 1):
            cur_xa = xa_values[i]
            next_xa = xa_values[i + 1]
            if (next_xa - cur_xa) / 2 >= args.xa_min_gap - 1e-3 or ignore_gap:
                for j in range(len(pivot_table.columns)):
                    if pd.isna(pivot_table.loc[cur_xa, pivot_table.columns[j]]) or pd.isna(pivot_table.loc[next_xa, pivot_table.columns[j]]):
                        continue
                    if str(pivot_table.loc[cur_xa, pivot_table.columns[j]])[0] != str(pivot_table.loc[next_xa, pivot_table.columns[j]])[0]:
                        xa_refine_values.add((cur_xa + next_xa) / 2)
                        break
        return xa_refine_values
    
    def get_xf_refine_values(self, ignore_gap=False):
        xf_refine_values = set()
        xf_values = self.get_xf_values()
        pivot_table = self.get_pivot_table()
        for i in range(len(xf_values) - 1):
            cur_xf = xf_values[i]
            next_xf = xf_values[i + 1]
            if (next_xf - cur_xf) / 2 >= args.xf_min_gap - 1e-3 or ignore_gap:
                for j in range(len(pivot_table.index)):
                    if pd.isna(pivot_table.loc[pivot_table.index[j], cur_xf]) or pd.isna(pivot_table.loc[pivot_table.index[j], next_xf]):
                        continue
                    if str(pivot_table.loc[pivot_table.index[j], cur_xf])[0] != str(pivot_table.loc[pivot_table.index[j], next_xf])[0]:
                        xf_refine_values.add((cur_xf + next_xf) / 2)
                        break
        return xf_refine_values
    
    def get_refine_values_for_largest_gap(self, parameter: Literal['xf', 'xa']):
        if parameter == 'xf':
            values = self.get_xf_values()
        elif parameter == 'xa':
            values = self.get_xa_values()
        else:
            raise ValueError(f'Invalid parameter: {parameter}')
        
        refined_values = set()
        largest_gap = 0
        for cur_val, next_val in zip(values[:-1], values[1:]):
            if next_val - cur_val > largest_gap and not eq(next_val - cur_val, largest_gap):
                largest_gap = next_val - cur_val
                refined_values = {(cur_val + next_val) / 2}
            elif eq(next_val - cur_val, largest_gap):
                refined_values.add((cur_val + next_val) / 2)
        return refined_values, largest_gap


    def min_xa(self):
        return min(self.get_xa_values())
    
    def max_xa(self):
        return max(self.get_xa_values())
    
    def min_xf(self):
        return min(self.get_xf_values())
    
    def max_xf(self):
        return max(self.get_xf_values())
    
    def all_cs_at_min_xa(self):
        return all([pd.isna(x) or (x == 'CS') for x in self.get_pivot_table().loc[self.min_xa()]])

    def all_ps_at_max_xa(self):
        return all([pd.isna(x) or (x == 'PS') for x in self.get_pivot_table().loc[self.max_xa()]])
    
    def all_cs_at_max_xa(self):
        return all([pd.isna(x) or (x == 'CS') for x in self.get_pivot_table().loc[self.max_xa()]])
    
    def all_cs_at_max_xf(self):
        return all([pd.isna(x) or (x == 'CS') for x in self.get_pivot_table().loc[:, self.max_xf()]])
    
    def same_for_two_max_xa(self):
        xa1, xa2 = self.get_xa_values()[-2:]
        xa_refine_values = self.get_xa_refine_values(True)
        if len(xa_refine_values) == 0:
            return True
        if xa1 <= max(xa_refine_values) <= xa2:
            return False
        return True
    
    def all_cs_at_min_xf(self):
        return all([pd.isna(x) or (x == 'CS') for x in self.get_pivot_table().loc[:, self.min_xf()]])
    
    def same_for_two_max_xf(self):
        xf1, xf2 = self.get_xf_values()[-2:]
        xf_refine_values = self.get_xf_refine_values(True)
        if len(xf_refine_values) == 0:
            return True
        if xf1 <= max(xf_refine_values) <= xf2:
            return False
        return True
        


def find_critical_value():
    for value in CRITICAL_VALUES[args.autopilot][args.vista_type]:
        if eq(value['ve'], args.ve):
            return value
    else:
        raise ValueError(f'No critical value found for ve={args.ve}')

critical_value = find_critical_value()

def fill_blank():
    unknown_data = result_manager.get_unknown_data()
    for data in unknown_data:
        if is_feasible(data):
            run_for(data)    

result_manager = ResultManager()
grid_result = ResultManager()

for xf in np.arange(args.min_xf, args.max_xf + args.xf_step / 2, args.xf_step):
    for xa in np.arange(args.min_xa, args.max_xa + args.xa_step / 2, args.xa_step):
        grid_result.add({'ve': args.ve, 'xa': xa, 'xf': xf, 'verdict': float('nan')})


xa_list = grid_result.get_xa_values()
xf_list = grid_result.get_xf_values()

xa1, xa2 = None, None
xf1, xf2 = None, None


# find the range of xa and xf that contains the critical value
if args.vista_type != 'crossing_with_traffic_lights':
    for i in range(len(xa_list) - 1):
        if xa_list[i+1] >= critical_value['xa'] >= xa_list[i]:
            xa1 = xa_list[i]
            xa2 = xa_list[i + 1]
            break

if critical_value['xf'] is not None:
    for i in range(len(xf_list) - 1):
        if xf_list[i+1] >= critical_value['xf'] >= xf_list[i]:
            xf1 = xf_list[i]
            xf2 = xf_list[i + 1]
            break
else:
    xf1 = xf_list[0]
    xf2 = xf_list[-1]

if args.vista_type != 'crossing_with_traffic_lights':
    assert xa1 is not None and xa2 is not None
assert xf1 is not None and xf2 is not None

if critical_value['xf'] is not None:
    run_for(critical_value)

if args.vista_type == 'crossing_with_traffic_lights':
    for xf in [xf1, xf2]:
        has_new = run_for({'ve': args.ve, 'xf': xf})
else:
    for xf in [xf1, xf2]:
        for xa in [xa1, xa2]:
            has_new = run_for({'ve': args.ve, 'xa': xa, 'xf': xf})

fill_blank()

def run_for_xa(xa, xfs=None):
    assert args.vista_type != 'crossing_with_traffic_lights'
    if xfs is None:
        xfs = result_manager.get_xf_values()
    for xf in xfs:
        has_new = run_for({'ve': args.ve, 'xa': xa, 'xf': xf}, refined_xa=xa)


def run_for_xf(xf, xas=None):
    if args.vista_type == 'crossing_with_traffic_lights':
        run_for({'ve': args.ve, 'xf': xf})
    else:
        if xas is None:
            xas = result_manager.get_xa_values()
        for xa in xas:
            has_new = run_for({'ve': args.ve, 'xa': xa, 'xf': xf}, refined_xf=xf)


if critical_value['xf'] is not None:
    run_for(critical_value)


def do_refinement():
    has_refined = False
    if args.vista_type != 'crossing_with_traffic_lights':
        xa_refine_values = grid_result.get_xa_refine_values()
        loguru.logger.info(f'Refining xa values: {xa_refine_values}')
        for xa in xa_refine_values:
            has_refined = True
            run_for_xa(xa)

    xf_refine_values = grid_result.get_xf_refine_values()
    loguru.logger.info(f'Refining xf values: {xf_refine_values}')
    for xf in xf_refine_values:
        has_refined = True
        run_for_xf(xf)
    return has_refined


def extend_min_xa():
    if not eq(result_manager.min_xa(), 0):
        extended_min_xa = max(result_manager.min_xa() - args.xa_step, args.min_xa)
        run_for_xa(extended_min_xa)
        return True
    return False
            
def extend_max_xa():
    if not eq(result_manager.max_xa(), args.max_xa):
        extended_max_xa = min(result_manager.max_xa() + args.xa_step, args.max_xa)
        run_for_xa(extended_max_xa)
        return True
    return False

def extend_min_xf():
    if not eq(result_manager.min_xf(), 0):
        extended_min_xf = max(result_manager.min_xf() - args.xf_step, args.min_xf)
        run_for_xf(extended_min_xf)
        return True
    return False
        
def extend_max_xf():
    if not eq(result_manager.max_xf(), args.max_xf):
        extended_max_xf = min(result_manager.max_xf() + args.xf_step, args.max_xf)
        run_for_xf(extended_max_xf)
        return True
    return False


while True:
    has_refined = do_refinement()
    
    if not has_refined:
        extended = False
        
        if args.vista_type != 'crossing_with_traffic_lights':
            if not result_manager.all_cs_at_min_xa() and not eq(result_manager.min_xa(), args.min_xa):
                loguru.logger.info(f'all_cs_at_min_xa is False, extending min_xa')
                extend_min_xa()
                extended = True
            
            if (not result_manager.same_for_two_max_xa() or result_manager.all_cs_at_max_xa()) and not eq(result_manager.max_xa(), args.max_xa):
                loguru.logger.info(f'same_for_two_max_xa is False, extending max_xa')
                extend_max_xa()
                extended = True

        if not result_manager.all_cs_at_min_xf() and not eq(result_manager.min_xf(), args.min_xf):
            loguru.logger.info(f'all_cs_at_min_xf is False, extending min_xf')
            extend_min_xf()
            extended = True
            
        if (not result_manager.same_for_two_max_xf() or result_manager.all_cs_at_max_xf()) and not eq(result_manager.max_xf(), args.max_xf):
            loguru.logger.info(f'same_for_two_max_xf is False, extending max_xf')
            extend_max_xf()
            extended = True
        
        if not extended:
            loguru.logger.info('No extension is needed by rules. Extend all boundaries')
            if args.vista_type != 'crossing_with_traffic_lights':
                extended |= extend_min_xa()
                extended |= extend_max_xa()
            extended |= extend_min_xf()
            extended |= extend_max_xf()
            
            if not extended:
                loguru.logger.info("Boundaries reach the limits. Refining large gaps")
                if args.vista_type != 'crossing_with_traffic_lights':
                    xa_refine_values, xa_largest_gap = result_manager.get_refine_values_for_largest_gap('xa')
                    xf_refine_values, xf_largest_gap = result_manager.get_refine_values_for_largest_gap('xf')
                    
                    if xa_largest_gap < args.xa_min_gap * 2 and xf_largest_gap < args.xf_min_gap * 2:
                        loguru.logger.info('All gaps are smaller than the minimum gap. Refining the largest gap')
                        if xf_largest_gap <= xa_largest_gap:
                            for xa in xa_refine_values:
                                run_for_xa(xa)
                        else:
                            for xf in xf_refine_values:
                                run_for_xf(xf)

                    if xa_largest_gap >= args.xa_min_gap * 2 and xf_largest_gap >= args.xf_min_gap * 2:
                        loguru.logger.info('All gaps are larger than the minimum gap. Refining the largest gap')
                
                        if xa_largest_gap >= xf_largest_gap:
                            for xa in xa_refine_values:
                                run_for_xa(xa)
                        else:
                            for xf in xf_refine_values:
                                run_for_xf(xf)

                    if xa_largest_gap < args.xa_min_gap * 2 and xf_largest_gap >= args.xf_min_gap * 2:
                        loguru.logger.info('Xf gaps are larger than the minimum gap. Refining the largest gap')
                        for xf in xf_refine_values:
                            run_for_xf(xf)
                            
                    if xa_largest_gap >= args.xa_min_gap * 2 and xf_largest_gap < args.xf_min_gap * 2:
                        loguru.logger.info('Xa gaps are larger than the minimum gap. Refining the largest gap')
                        for xa in xa_refine_values:
                            run_for_xa(xa)
                            
                else:
                    xf_refine_values, xf_largest_gap = result_manager.get_refine_values_for_largest_gap('xf')
                    loguru.logger.info('Refining the largest gap')
                    for xf in xf_refine_values:
                        run_for_xf(xf)