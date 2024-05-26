import json
from src.simulator.autoware.autoware_vehicle_state import AutowareVehicleState

main_verdict = 'unknown'
safety_issues = []

raw = ''
while True:
    try:
        line = input()
        raw += line
    except EOFError:
        break
    
data = json.loads(raw)

def dis(a, b):
    return ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5

if data['collision'] == 'yes':
    main_verdict = 'collision'
    last_ego_state = AutowareVehicleState(data['ego_trace'][-1])
    last_arriving_state = AutowareVehicleState(data['arriving_trace'][-1])
    
    last_ego_center = last_ego_state.geometry_center
    last_ego_front = last_ego_state.front_position
    last_arriving_center = last_arriving_state.geometry_center
    last_arriving_front = last_arriving_state.front_position
    if dis(last_ego_front, last_arriving_center) < dis(last_arriving_front, last_ego_center):
        safety_issues.append('ego_fault')
    else:
        safety_issues.append('arriving_fault')
else:
    last_ego_state = AutowareVehicleState(data['ego_trace'][-1])
    last_arriving_state = AutowareVehicleState(data['arriving_trace'][-1])
    
    if last_arriving_state.y < last_ego_state.y or abs(last_ego_state.x - -1.75) > 1.75:
        main_verdict = 'caution'
    else:
        main_verdict = 'progress'
            
report = {
    'xe': data['xe'],
    've': data['ve'],
    'xf': data['xf'],
    'xa': data['xa'],
    'verdict': main_verdict,
    'safety_issues': safety_issues
}

print(json.dumps(report))
