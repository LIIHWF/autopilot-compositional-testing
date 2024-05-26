import json
import sys


data = json.load(sys.stdin)


main_verdict = 'unknown'

safety_issues = []


def dis(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5


if data['collision'] == 'yes':
    main_verdict = 'collision'
    ego_position = data['state_sequence'][-1]['ego']['transform']['location']
    arriving_position = data['state_sequence'][-1]['arriving']['transform']['location']
    ego_front = ego_position['x'] + \
        data['state_sequence'][-1]['ego']['shape']['x'], ego_position['y']
    arriving_front = arriving_position['x'], arriving_position['y'] + \
        data['state_sequence'][-1]['arriving']['shape']['x']

    ego_position = ego_position['x'], ego_position['y']
    arriving_position = arriving_position['x'], arriving_position['y']

    if dis(ego_front, arriving_position) < dis(arriving_front, ego_position):

        safety_issues.append('ego_fault')
    else:
        safety_issues.append('arriving_fault')
else:
    for state in data['state_sequence']:
        ego_state = state['ego']
        arriving_state = state['arriving']
        if main_verdict == 'unknown':
            if ego_state['transform']['location']['x'] > -1.75:
                main_verdict = 'progress'
            elif arriving_state['transform']['location']['y'] > 1.75:
                main_verdict = 'caution'
        if -12 < ego_state['transform']['location']['x'] + ego_state['shape']['x'] < 12:
            if -12 < arriving_state['transform']['location']['y'] + arriving_state['shape']['x'] < 12:
                if 'BOTH_IN_JUNCTION' not in safety_issues:
                    safety_issues.append('BOTH_IN_JUNCTION')
            if ego_state['velocity']['x'] < 1e-5:
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
