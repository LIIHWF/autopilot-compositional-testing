import json
from src.simulator.apollo.apollo_world_model import ApolloWorldModel

main_verdict = 'unknown'
safety_issues = []


EGO_ID = 'apollo-1'
ARRIVING_ID = 'apollo-2'
FRONT_ID = 'static-10'

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

if data['collision'] == 'yes':
    main_verdict = 'collision'
    
for state in data['state_sequence']:
    state = ApolloWorldModel.from_dict(state)
    if main_verdict == 'unknown' and state.vehicles_states[ARRIVING_ID].x > 0:
        main_verdict = 'caution'
    if main_verdict == 'unknown' and state.vehicles_states[EGO_ID].y > -3.5:
        main_verdict = 'progress'

    if (-12 < state.vehicles_states[EGO_ID].front_position.y < 12 and
        -12 < state.vehicles_states[ARRIVING_ID].front_position.x < 12):
        if 'BOTH_IN_JUNCTION' not in safety_issues:
            safety_issues.append('BOTH_IN_JUNCTION')
        
    if (-12 < state.vehicles_states[EGO_ID].front_position.y < 12 and 
        state.vehicles_states[EGO_ID].v < 1e-6):
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