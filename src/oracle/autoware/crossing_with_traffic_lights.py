import json
import loguru
from src.simulator.autoware import AutowareVehicleState

raw = ''
while True:
    try:
        line = input()
        raw += line
    except EOFError:
        break
    
# parse the JSON
data = json.loads(raw)

main_verdict = 'caution'
safety_issues = []
pre_inside_junction = False

for ego_state, self_color, side_color in zip(data['ego_trace'], data['self_light_trace'], data['side_light_trace']):
    ego_state = AutowareVehicleState(ego_state)
    if ego_state.y > 0:
        main_verdict = 'progress'
    
    loguru.logger.info(f'{ego_state.front_position.y}, {self_color}, {side_color}')
    
    if -12 < ego_state.front_position.y < 12:
        if self_color == 'Red' and not pre_inside_junction:
            if 'RUN_RED_LIGHT' not in safety_issues:
                safety_issues.append('RUN_RED_LIGHT')
        if side_color == 'Green':
            if 'IN_JUNCTION_WHEN_SIDE_GREEN' not in safety_issues:
                safety_issues.append('IN_JUNCTION_WHEN_SIDE_GREEN')
        pre_inside_junction = True
    else:
        pre_inside_junction = False
        
if main_verdict == 'caution':
    last_ego_state = AutowareVehicleState(data['ego_trace'][-1])
    if -12 < last_ego_state.front_position.y < 12:
        if 'EGO_STOPPED' not in safety_issues:
            safety_issues.append('EGO_STOPPED')

if main_verdict == 'progress':
    for ego_state in data['ego_trace']:
        ego_state = AutowareVehicleState(ego_state)
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