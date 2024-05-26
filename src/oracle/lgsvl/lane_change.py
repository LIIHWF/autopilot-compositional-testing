import json
import sys
import math
import loguru

data = json.load(sys.stdin)


main_verdict = 'unknown'

safety_issues = []


def dis(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5


half_length = 2.27881813049316

main_verdict = 'unknown'
ego_last_state = data['state_sequence'][-1]['ego']
arriving_last_state = data['state_sequence'][-1]['arriving']

ego_center_position = (ego_last_state['transform']['position']['x'],
                        ego_last_state['transform']['position']['z'])
arriving_center_position = arriving_last_state['transform']['position']['x'], \
    arriving_last_state['transform']['position']['z']

ego_forward_vector = math.sin(math.radians(ego_last_state['transform']['rotation']['y'])), \
    math.cos(math.radians(ego_last_state['transform']['rotation']['y']))

arriving_forward_vector = math.sin(math.radians(arriving_last_state['transform']['rotation']['y'])), \
    math.cos(math.radians(arriving_last_state['transform']['rotation']['y']))

ego_front_position = (ego_center_position[0] + half_length * ego_forward_vector[0],
                        ego_center_position[1] + half_length * ego_forward_vector[1])
arriving_front_position = (arriving_center_position[0] + half_length * arriving_forward_vector[0],
                            arriving_center_position[1] + half_length * arriving_forward_vector[1])
if data['result'] == 'collision':
    main_verdict = 'collision'
    if dis(ego_front_position, arriving_center_position) < dis(arriving_front_position, ego_center_position):
        safety_issues.append('ego_fault')
    else:
        safety_issues.append('arriving_fault')
else:
    # last_ego_state = data['state_sequence'][-1]['ego']
    # last_arriving_state = data['state_sequence'][-1]['arriving']    
    # if abs(ego_last_state['transform']['position']['x'] - -1.75) < 1.75 and \

    if ego_front_position[0] < -0.3 and \
            ego_last_state['transform']['position']['z'] > arriving_last_state['transform']['position']['z']:
        main_verdict = 'progress'
    else:
        main_verdict = 'caution'

report = {
    'xf\'': data['xf\''],
    've': data['ve'],
    'xf': data['xf'],
    'xa': data['xa'],
    'verdict': main_verdict,
    'safety_issues': safety_issues
}

print(json.dumps(report))
