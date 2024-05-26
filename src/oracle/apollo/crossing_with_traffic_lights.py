import json
from src.simulator.apollo.apollo_world_model import ApolloWorldModel
import loguru

main_verdict = 'unknown'
safety_issues = []


EGO_ID = 'apollo-1'
ARRIVING_ID = 'apollo-2'
FRONT_ID = 'static-10'

# read stdin until EOF
raw = ''
while True:
    try:
        line = input()
        raw += line
    except EOFError:
        break
    
# parse the JSON
data = json.loads(raw)

if data['collision'] == 'yes':
    main_verdict = 'collision'
    
pre_inside_junction = False
self_traffic_light_id = 'sign_0'
side_traffic_light_id = 'sign_1'

main_verdict = 'caution'

for state in data['state_sequence']:
    state = ApolloWorldModel.from_dict(state)
    ego_state = state.vehicles_states[EGO_ID]
    
    if ego_state.y > 0:
        main_verdict = 'progress'

    loguru.logger.info(f'{ego_state.front_position.y}, {state.traffic_lights[self_traffic_light_id]}, {state.traffic_lights[side_traffic_light_id]}')

    if -12 < ego_state.front_position.y < 12:
        if not pre_inside_junction and state.traffic_lights[self_traffic_light_id].name == 'RED':
            if 'RUN_RED_LIGHT' not in safety_issues:
                safety_issues.append('RUN_RED_LIGHT')
        if state.traffic_lights[side_traffic_light_id].name == 'GREEN':
            if 'IN_JUNCTION_WHEN_SIDE_GREEN' not in safety_issues:
                safety_issues.append('IN_JUNCTION_WHEN_SIDE_GREEN')

        pre_inside_junction = True
    else:
        pre_inside_junction = False
        
if main_verdict == 'caution':
    last_ego_state = ApolloWorldModel.from_dict(data['state_sequence'][-1]).vehicles_states[EGO_ID]
    if -12 < last_ego_state.front_position.y < 12:
        if 'EGO_STOPPED' not in safety_issues:
                safety_issues.append('EGO_STOPPED')

if main_verdict == 'progress':
    for ego_state in data['state_sequence']:
        ego_state = ApolloWorldModel.from_dict(ego_state).vehicles_states[EGO_ID]
        if -12 < ego_state.front_position.y < 12:
            if ego_state.v < 0.5 and (not data['ve'] == 0 or -10 < ego_state.front_position.y) and (not data['xf'] < 2 or ego_state.front_position.y < 10):
                    if 'EGO_STOPPED' not in safety_issues:
                        safety_issues.append('EGO_STOPPED')

report = {
    'xe': data['xe'],
    've': data['ve'],
    'xf': data['xf'],
    'verdict': main_verdict,
    'safety_issues': safety_issues
}

print(json.dumps(report))