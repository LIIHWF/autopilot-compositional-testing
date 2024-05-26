import json
import sys
import loguru

data = json.load(sys.stdin)


main_verdict = 'caution'

safety_issues = []


def dis(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

pre_inside_junction = False
for ego_state, self_light_color, side_light_color in zip(data['ego_trace'], data['self_traffic_light_trace'], data['side_traffic_light_trace']):
    if main_verdict == 'caution':
        if ego_state['transform']['location']['x'] > 0:
            main_verdict = 'progress'
    if -12 < ego_state['transform']['location']['x'] + ego_state['shape']['x'] < 12:
        if ego_state['velocity']['x'] < 1e-5:
            if 'EGO_STOPPED' not in safety_issues:
                safety_issues.append('EGO_STOPPED')
        if not pre_inside_junction and self_light_color == 'Red':
            if 'RUN_RED_LIGHT' not in safety_issues:
                safety_issues.append('RUN_RED_LIGHT')
        if side_light_color == 'Green':
            if 'IN_JUNCTION_WHEN_SIDE_GREEN' not in safety_issues:
                safety_issues.append('IN_JUNCTION_WHEN_SIDE_GREEN')
        pre_inside_junction = True
    else:
        pre_inside_junction = False

if main_verdict == 'caution':
    last_ego_state = data['ego_trace'][-1]
    if -12 < last_ego_state['transform']['location']['x'] + last_ego_state['shape']['x'] < 12:
        if 'EGO_STOPPED' not in safety_issues:
            safety_issues.append('EGO_STOPPED')

if main_verdict == 'progress':
    for ego_state in data['ego_trace']:
        front_x = ego_state['transform']['location']['x'] + ego_state['shape']['x']
        if -12 < front_x < 12:
            if ego_state['velocity']['x'] < 0.5 and (not data['xe'] == 0 or -10 < front_x) and (not data['xf'] < 2 or front_x < 10):
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
