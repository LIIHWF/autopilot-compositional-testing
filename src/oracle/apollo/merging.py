import json
from src.simulator.apollo import ApolloWorldModel
import sys
import time

main_verdict = 'unknown'
safety_issues = []


EGO_ID = 'apollo-1'
ARRIVING_ID = 'apollo-2'
FRONT_ID = 'static-10'


data = json.load(sys.stdin)

if data['collision'] == 'yes':
    main_verdict = 'collision'
elif data['planning_failed'] == 'yes':
    main_verdict = 'planning_failed'


if main_verdict == 'unknown':
    last_state = ApolloWorldModel.from_dict(data['state_sequence'][-1])
    ego_state = last_state.vehicles_states[EGO_ID]
    arriving_state = last_state.vehicles_states[ARRIVING_ID]
    if ego_state.y < arriving_state.y and ego_state.front_position.x > 0.3:
        main_verdict = 'progress'
    else:
        main_verdict = 'caution'
if main_verdict == 'unknown':
    main_verdict = 'caution'


def distance(p1, p2):
    return ((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2) ** 0.5


if main_verdict == 'collision':
    last_state = ApolloWorldModel.from_dict(data['state_sequence'][-1])
    ego_state = last_state.vehicles_states[EGO_ID]
    arriving_state = last_state.vehicles_states[ARRIVING_ID]
    if distance(ego_state.front_position, arriving_state.vehicle_center) < distance(arriving_state.front_position, ego_state.vehicle_center):
        safety_issues.append('ego_fault')
    else:
        safety_issues.append('arriving_fault')

report = {
    'xe': data['xe'],
    've': data['ve'],
    'xf': data['xf'],
    'xa': data['xa'],
    'verdict': main_verdict,
    'safety_issues': safety_issues
}

print(json.dumps(report))
