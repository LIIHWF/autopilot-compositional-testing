import json
import sys


data = json.load(sys.stdin)


main_verdict = 'unknown'

safety_issues = []


def dis(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5


main_verdict = 'unknown'
if data['result'] == 'collision':
    main_verdict = 'collision'
    ego_last_state = data['state_sequence']['ego'][-1]
    arriving_last_state = data['state_sequence']['arriving'][-1]
    ego_front_position = (ego_last_state['physical_state']['position']['x'], ego_last_state['physical_state']['position']['z'])
    ego_forward_vector = (ego_last_state['physical_state']['forward']['x'], ego_last_state['physical_state']['forward']['z'])
    arriving_front_position = (arriving_last_state['physical_state']['position']['x'], arriving_last_state['physical_state']['position']['z'])
    arriving_forward_vector = (arriving_last_state['physical_state']['forward']['x'], arriving_last_state['physical_state']['forward']['z'])
    ego_center_position = (ego_front_position[0] - 2 * ego_forward_vector[0], ego_front_position[1] - 2 * ego_forward_vector[1])
    arriving_center_position = (arriving_front_position[0] - 2 * arriving_forward_vector[0], arriving_front_position[1] - 2 * arriving_forward_vector[1])
    if dis(ego_front_position, arriving_center_position) < dis(arriving_front_position, ego_center_position):
        safety_issues.append('ego_fault')
    else:
        safety_issues.append('arriving_fault')
else:
    for ego_state, arriving_state in zip(data['state_sequence']['ego'], data['state_sequence']['arriving']):
        if main_verdict == 'unknown':
            if ego_state['lane_id'] == 'lane_13' and ego_state['lane_offset'] > 12:
                main_verdict = 'progress'
            elif arriving_state['lane_id'] == 'lane_14' and arriving_state['lane_offset'] > 12 + 3.5:
                main_verdict = 'caution'
        if ego_state['lane_id'] == 'lane_13' and arriving_state['lane_id'] == 'lane_14':
            if 'BOTH_IN_JUNCTION' not in safety_issues:
                safety_issues.append('BOTH_IN_JUNCTION')
        if ego_state['lane_id'] == 'lane_13' and ego_state['control_speed'] < 1e-4:
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
