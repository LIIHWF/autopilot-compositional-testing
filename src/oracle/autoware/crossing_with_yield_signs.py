import json
import loguru
from src.simulator.autoware import AutowareVehicleState

main_verdict = 'unknown'
safety_issues = []


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

def dis(a, b):
    return ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5

if data['collision'] == 'yes':
    main_verdict = 'collision'
    last_ego_state = AutowareVehicleState(data['ego_trace'][-1])
    last_arriving_state = AutowareVehicleState(data['arriving_trace'][-1])
    
    ego_state_half_sec_ago = AutowareVehicleState(data['ego_trace'][-5])
    
    last_ego_center = last_ego_state.geometry_center
    last_ego_front = last_ego_state.front_position
    last_arriving_center = last_arriving_state.geometry_center
    last_arriving_front = last_arriving_state.front_position
    loguru.logger.info(f'{last_ego_front.x, last_ego_front.y}, {last_ego_center.x, last_ego_center.y} ? {last_arriving_front.x, last_arriving_front.y}, {last_arriving_center.x, last_arriving_center.y}')
    loguru.logger.info(f'{dis(last_ego_front, last_arriving_center)} ? {dis(last_arriving_front, last_ego_center)}')
    if abs(dis(last_ego_front, last_arriving_center) - dis(last_arriving_front, last_ego_center)) < 0.1 and ego_state_half_sec_ago.v < 1e-4:
        safety_issues.append('arriving_fault')
    elif dis(last_ego_front, last_arriving_center) < dis(last_arriving_front, last_ego_center):
        safety_issues.append('ego_fault')
    else:
        safety_issues.append('arriving_fault')
else:
    for ego_state, arriving_state in zip(data['ego_trace'], data['arriving_trace']):
        ego_state = AutowareVehicleState(ego_state)
        arriving_state = AutowareVehicleState(arriving_state)
        if main_verdict == 'unknown' and ego_state.y > 0:
            main_verdict = 'progress'
        if main_verdict == 'unknown' and arriving_state.x > 3.5:
            main_verdict = 'caution'

        if (-12 < ego_state.front_position.y < 12 and
            -12 < arriving_state.front_position.x < 12):
            if 'BOTH_IN_JUNCTION' not in safety_issues:
                safety_issues.append('BOTH_IN_JUNCTION')
            
        if (-12 < ego_state.front_position.y < 12 and 
            ego_state.v < 1e-5):
            # print(state.vehicles_states[EGO_ID].front_position.y)
            if 'EGO_STOPPED' not in safety_issues:
                safety_issues.append('EGO_STOPPED')
            
report = {
    'xe': data['xe'],
    've': data['ve'],
    'xf': data['xf'],
    'xa': data['xa'],
    'verdict': main_verdict,
    'safety_issues': safety_issues
}

print(json.dumps(report))