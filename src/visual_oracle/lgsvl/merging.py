import json
import sys


data = json.load(sys.stdin)


main_verdict = 'unknown'

safety_issues = []


def dis(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5


main_verdict = 'unknown'

half_length = 2.27881813049316

if data['result'] == 'collision':
    main_verdict = 'collision'
    ego_last_state = data['state_sequence']['ego'][-1]
    arriving_last_state = data['state_sequence']['arriving'][-1]

    ego_center_position = (ego_last_state['physical_state']['position']
                           ['x'], ego_last_state['physical_state']['position']['z'])
    ego_forward_vector = (ego_last_state['physical_state']['forward']
                          ['x'], ego_last_state['physical_state']['forward']['z'])
    arriving_center_position = (
        arriving_last_state['physical_state']['position']['x'], arriving_last_state['physical_state']['position']['z'])
    arriving_forward_vector = (
        arriving_last_state['physical_state']['forward']['x'], arriving_last_state['physical_state']['forward']['z'])
    ego_front_position = (
        ego_center_position[0] + half_length * ego_forward_vector[0], ego_center_position[1] + half_length * ego_forward_vector[1])
    arriving_front_position = (
        arriving_center_position[0] + half_length * arriving_forward_vector[0], arriving_center_position[1] + half_length * arriving_forward_vector[1])
    if dis(ego_front_position, arriving_center_position) < dis(arriving_front_position, ego_center_position):
        safety_issues.append('ego_fault')
    else:
        safety_issues.append('arriving_fault')
else:
    last_ego_state = data['state_sequence']['ego'][-1]
    last_arriving_state = data['state_sequence']['arriving'][-1]
    if abs(last_ego_state['physical_state']['position']['z'] - -1.75) < 1.75 and \
            last_ego_state['physical_state']['position']['x'] > last_arriving_state['physical_state']['position']['x']:
        main_verdict = 'progress'
    else:
        main_verdict = 'caution'


report = {
    'xe': data['xe'],
    've': data['ve'],
    'xf': data['xf'],
    'xa': data['xa'],
    'verdict': main_verdict,
    'safety_issues': safety_issues
}

print(json.dumps(report))
