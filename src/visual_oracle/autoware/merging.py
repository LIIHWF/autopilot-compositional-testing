import json
from src.simulator.autoware.autoware_vehicle_state import AutowareVehicleState
from src.dynamics.autoware_dynamics import AutowareDynamics
import matplotlib.pyplot as plt
import argparse
import os
import sys
import io

parser = argparse.ArgumentParser()
parser.add_argument('-o', '--output', help='Output file name')
parser.add_argument('-p', '--progress', action='store_true', help='Show progress')
parser.add_argument('-t', '--tunnel', action='store_true', help='Show tunnel')
args = parser.parse_args()

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

ego_trace = [AutowareVehicleState(state) for state in data['ego_trace']]

traveld_distance = [0]
speed = [ego_trace[0].v]

def delta_distance(a, b):
    return ((a.x - b.x) ** 2 + (a.y - b.y) ** 2) ** 0.5

for i in range(1, len(ego_trace)):
    traveld_distance.append(traveld_distance[-1] + delta_distance(ego_trace[i], ego_trace[i - 1]))
    speed.append(ego_trace[i].v)
    
# set figure size
plt.figure(figsize=(7, 2))    

plt.plot(traveld_distance, speed, 'black', linewidth=2, label='ego vehicle\'s trace')

if collision := data['collision'] == 'yes':
    # mar a cross
    x, y  = traveld_distance[-1], speed[-1]
    if y < 0.5:
        y = 0.5
    plt.plot(x, y, 'rX', markersize=10, label='accident')

if os.path.exists(f'data/policy/caution/autoware/ve={round(data["ve"], 1)}.json'):
    with open(f'data/policy/caution/autoware/ve={round(data["ve"], 1)}.json') as f:
        policy = json.load(f)
        caution_distance_list = [p[0] for p in policy]
        caution_speed_list = [p[1] for p in policy]
        # linewith to bolder
        # make the line not be full occupied
        plt.plot(caution_distance_list, caution_speed_list, 'b--', linewidth=3, alpha=0.5, label='feasible cautious policy')

with open('test.json', 'w') as f:
    f.write(json.dumps(list(zip(traveld_distance, speed)), indent=4))
    
if args.progress:
    xs, vs = AutowareDynamics().fast_trace(data['ve'], data['xf'] + data['xe'])
    plt.plot(xs, vs, 'g--', linewidth=3, alpha=0.5, label='feasible progressive policy')

# draw a line at data['xe'] on x-axis
plt.axvline(x=data['xe'] + 0.1 * data['ve'], color='gray', linestyle='--', label='merge point', linewidth=2)

# report = {
#     'xe': data['xe'],
#     've': data['ve'],
#     'xf': data['xf'],
#     'xa': data['xa'],
#     'verdict': main_verdict,
#     'safety_issues': safety_issues
# }

# print(json.dumps(report))
#  make a legend at right down coner outside the plot

plt.xlabel('Traveled distance (m)')
plt.ylabel('Speed (m/s)')

plt.xlim(0)
plt.ylim(0)

plt.legend(loc='lower left', bbox_to_anchor=(1, -0.05))
plt.tight_layout()

if args.tunnel:
    buf = io.BytesIO()
    plt.savefig(buf, format='png', dpi=400)
    buf.seek(0)
    plt.close()
    sys.stdout.buffer.write(buf.read())

if args.output:
    plt.savefig(args.output, dpi=400)
elif not args.tunnel:
    plt.show()