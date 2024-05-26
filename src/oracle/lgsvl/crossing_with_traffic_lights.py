import json
import sys
import loguru

data = json.load(sys.stdin)

half_length = 2.27881813049316
main_verdict = 'caution'

safety_issues = []


pre_inside_junction = False
i = 0
for ego_state, self_light_color, side_light_color in zip(data['ego_trace'], data['self_traffic_light_trace'], data['side_traffic_light_trace']):
    ego_front = ego_state['physical_state']['position']['z'] - half_length
    if main_verdict == 'caution':
        if ego_front < 0:
            main_verdict = 'progress'
    if i < 600:
        loguru.logger.info(f'ego_front: {ego_front}, self_light_color: {self_light_color}, side_light_color: {side_light_color}, i: {i}')
    i += 1
    if -12 < ego_front < 12:
        if not pre_inside_junction and self_light_color == 'red':
            if 'RUN_RED_LIGHT' not in safety_issues:
                safety_issues.append('RUN_RED_LIGHT')
        if side_light_color == 'green':
            if 'IN_JUNCTION_WHEN_SIDE_GREEN' not in safety_issues:
                safety_issues.append('IN_JUNCTION_WHEN_SIDE_GREEN')
        pre_inside_junction = True
    else:
        pre_inside_junction = False

if main_verdict == 'caution':
    last_ego_state = data['ego_trace'][-1]
    if -12 < last_ego_state['physical_state']['position']['z'] - half_length < 12:
        if 'EGO_STOPPED' not in safety_issues:
            safety_issues.append('EGO_STOPPED')

if main_verdict == 'progress':
    for ego_state in data['ego_trace']:
        ego_front = ego_state['physical_state']['position']['z'] - half_length
        if -12 < ego_front < 12:
            if ego_state['control_speed'] < 0.5 and (not data['xe'] == 0 or -10 < ego_front) and (not data['xf'] < 2 or ego_front < 10):
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
