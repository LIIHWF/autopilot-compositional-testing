import json
from src.simulator.apollo.apollo_world_model import ApolloWorldModel
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
    last_state = ApolloWorldModel.from_dict(data['state_sequence'][-1])
    ego_state = last_state.vehicles_states[EGO_ID]
    arriving_state = last_state.vehicles_states[ARRIVING_ID]
    if arriving_state.x > ego_state.x + 6:
        main_verdict = 'caution'
    else:
        main_verdict = 'planning_failed'


if main_verdict == 'unknown':
    for state in data['state_sequence']:
        state = ApolloWorldModel.from_dict(state)
        if main_verdict == 'unknown':
            if abs(state.vehicles_states[EGO_ID].y - 1.75) < 0.5 and abs(state.vehicles_states[ARRIVING_ID].y - 1.75) < 0.5:
                if state.vehicles_states[EGO_ID].x > state.vehicles_states[ARRIVING_ID].x:
                    main_verdict = 'progress'
if main_verdict == 'unknown':
    main_verdict = 'caution'

last_state = ApolloWorldModel.from_dict(data['state_sequence'][-1])
last_ego_state = last_state.vehicles_states[EGO_ID]
last_arriving_state = last_state.vehicles_states[ARRIVING_ID]
front_state = last_state.vehicles_states[FRONT_ID]

if main_verdict in ['caution', 'progress']:
    if last_ego_state.x > last_arriving_state.x and front_state.x - last_ego_state.x > 15 and last_ego_state.front_position.y > 0:
        main_verdict = 'blocking'


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
    'xf\'': data['xf\''],
    've': data['ve'],
    'xf': data['xf'],
    'xa': data['xa'],
    'verdict': main_verdict,
    'safety_issues': safety_issues
}

print(json.dumps(report))
