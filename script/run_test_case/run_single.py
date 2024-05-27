import argparse
import subprocess
import json
import loguru


parser = argparse.ArgumentParser('Run a single test case')

# vista_type is taken from { merging , lane_change , crossing_with_yield_signs , crossing_with_traffic_lights }
parser.add_argument('autopilot', type=str, help='autopilot to test', choices=['apollo', 'autoware', 'carla', 'lgsvl'])
parser.add_argument('vista_type', type=str, help='type of vista', choices=['merging', 'lane_change', 'crossing_with_yield_signs', 'crossing_with_traffic_lights'])
parser.add_argument('-ve', required=True, type=float, help='speed of the ego vehicle')
parser.add_argument('-xf', required=True, type=float, help='distance from the critical zone to the front vehicle')
parser.add_argument('-xa', required=False, type=float, help='distance from the arriving vehicle to the critical zone')
parser.add_argument('--log', action='store_true', help='enable logging', default=False)
parser.add_argument('--json', action='store_true', help='output in json format', default=False)

args = parser.parse_args()


ISSUE_MAP = {
    'BOTH_IN_JUNCTION': 'p1',
    'EGO_STOPPED': 'p2',
    'RUN_RED_LIGHT': 'p3',
    'IN_JUNCTION_WHEN_SIDE_GREEN': 'p4'
}

def simple_verdict(result):
    if result['verdict'] == 'collision':
        verdict = 'A'
    elif result['verdict'] == 'progress':
        verdict = 'P'
    elif result['verdict'] == 'caution':
        verdict = 'C'
    elif result['verdict'] == 'planning_failed':
        verdict = 'Fsw'
    elif result['verdict'] == 'blocking':
        verdict = 'Blk'
    else:
        raise ValueError(f'Unknown verdict: {result["verdict"]}')
    if verdict == 'A':
        if 'ego_fault' in result['safety_issues']:
            verdict += 'e'
        if 'arriving_fault' in result['safety_issues']:
            verdict += 'a'
    elif verdict in ['P', 'C']:
        if len(result['safety_issues']) == 0:
            verdict += 'S'
        else:
            verdict += 'U'
            for issue in ISSUE_MAP:
                if issue in result['safety_issues']:
                    verdict += ISSUE_MAP[issue]
    return verdict


PROPERTY_STR = {
    'p1': 'Two vehicles must not be in the critical zone at the same time.',
    'p2': 'The ego vehicle must not stop inside the critical zone.',
    'p3': 'The ego vehicle must not enter the critical zone when the light is red.',
    'p4': 'The ego vehicle must not be in the critical zone when a side light is green.'
}


FINELY_TUNED_XE = {
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
    'lgsvl': {
        'merging': {0.0: 0.0, 5.0: 1.3, 10.0: 2.6, 15.0: 3.8},
        'lane_change': {5.0: 1.3, 10.0: 2.6, 15.0: 3.8, 20.0: 5.1},
        'crossing_with_yield_signs': {0.0: 0.05, 5.0: 1.3, 10.0: 2.6, 15.0: 3.8},
        'crossing_with_traffic_lights': {0.0: 0.05, 5.0: 1.3, 10.0: 2.6, 15.0: 3.8, 20.0: 5.1}
    }
}

if args.vista_type in ['merging', 'crossing_with_yield_signs']:
    command = f'bazel-bin/src/simulator/adapter/{args.autopilot}/{args.vista_type} -ve {args.ve} -xe {FINELY_TUNED_XE[args.autopilot][args.vista_type][args.ve]} -xf {args.xf} -xa {args.xa}'
elif args.vista_type == 'lane_change':
    command = f'bazel-bin/src/simulator/adapter/{args.autopilot}/{args.vista_type} -ve {args.ve} -xff {FINELY_TUNED_XE[args.autopilot][args.vista_type][args.ve]} -xf {args.xf} -xa {args.xa}'
elif args.vista_type == 'crossing_with_traffic_lights':
    command = f'bazel-bin/src/simulator/adapter/{args.autopilot}/{args.vista_type} -ve {args.ve} -xe {FINELY_TUNED_XE[args.autopilot][args.vista_type][args.ve]} -xf {args.xf}'
else:
    raise ValueError(f'Invalid vista type: {args.vista_type}')

# command += f' | bazel-bin/src/oracle/{args.autopilot}/{args.vista_type}'

oracle_command = f'bazel-bin/src/oracle/{args.autopilot}/{args.vista_type}'

# result = subprocess.run(command, shell=True, stdout=subprocess.PIPE, stderr=(None if args.log else subprocess.PIPE), text=True)
result = subprocess.run(command.split(), shell=False, stdout=subprocess.PIPE, stderr=(None if args.log else subprocess.PIPE), text=True)
try:
    # result = json.loads(result.stdout)
    result = subprocess.run(
        oracle_command.split(), shell=False, 
        stdout=subprocess.PIPE, stderr=(None if args.log else subprocess.PIPE), text=True, input=result.stdout)
    result = json.loads(result.stdout)
except json.JSONDecodeError:
    loguru.logger.error('Error occur during the simulation. Please retry or add --log option for more information.')
    exit()

if not args.json:
    print()
    print('================== Report ==================')
    verdict = simple_verdict(result)
    print('Verdict:', verdict)
    print('Parameter:')
    for key in ['ve', 'xf', 'xa']:
        if key in result:
            print(f'    {key}:', result[key])
    if len(result['safety_issues']) > 0 and 'A' not in verdict:
        print('Safety violations:')
        for p in ['p1', 'p2', 'p3', 'p4']:
            if p in verdict:
                print(f'    {p}:', PROPERTY_STR[p])
    print()
else:
    print(json.dumps({
        've': args.ve,
        'xf': args.xf,
        'xa': args.xa,
        'verdict': simple_verdict(result),
    }))