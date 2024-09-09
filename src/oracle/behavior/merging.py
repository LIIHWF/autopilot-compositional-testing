import json
import sys
import math
import loguru


data = json.load(sys.stdin)


main_verdict = 'unknown'

safety_issues = []


def dis(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5


if data['collision'] == 'yes':
    main_verdict = 'collision'
    ego_last_state = data['state_sequence'][-1]['ego']
    arriving_last_state = data['state_sequence'][-1]['arriving']
    
    ego_position = data['state_sequence'][-1]['ego']['transform']['location']
    arriving_position = data['state_sequence'][-1]['arriving']['transform']['location']
    ego_position = ego_position['x'], ego_position['y']
    arriving_position = arriving_position['x'], arriving_position['y']
    
    ego_front = (
        ego_position[0] + math.cos(math.radians(ego_last_state['transform']
                                   ['rotation']['yaw'])) * ego_last_state['shape']['x'],
        ego_position[1] + math.sin(math.radians(ego_last_state['transform']['rotation']['yaw'])) * ego_last_state['shape']['x'])
    arriving_front = (
        arriving_position[0] + math.cos(math.radians(arriving_last_state['transform']
                                        ['rotation']['yaw'])) * arriving_last_state['shape']['x'],
        arriving_position[1] + math.sin(math.radians(arriving_last_state['transform']
                                        ['rotation']['yaw'])) * arriving_last_state['shape']['x']
    )
    loguru.logger.info(f'{ego_front} {arriving_front} {ego_position} {arriving_position} {arriving_last_state["transform"]["rotation"]["yaw"]} {ego_last_state["transform"]["rotation"]["yaw"]}')
    loguru.logger.info(f'{dis(ego_front, arriving_position)} {dis(arriving_front, ego_position)}')
    if dis(ego_front, arriving_position) < dis(arriving_front, ego_position):
        safety_issues.append('ego_fault')
    else:
        safety_issues.append('arriving_fault')
else:
    last_state = data['state_sequence'][-1]
    last_ego_state = last_state['ego']
    last_arriving_state = last_state['arriving']
    if abs(last_ego_state['transform']['location']['y'] - 1.75) < 1.75 and last_ego_state['transform']['location']['x'] > last_arriving_state['transform']['location']['x']:
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
