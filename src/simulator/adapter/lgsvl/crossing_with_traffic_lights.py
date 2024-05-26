import lgsvl
import loguru
from src.simulator.adapter.argument import make_arg_parser
from src.simulator.context import Context
import json

experiment_args = make_arg_parser('lgsvl', 'crossing_with_traffic_lights').parse_args()

def translate_policy(policy_list, translate_seconds):
    if translate_seconds == 0:
        return policy_list
    state, wait = policy_list[0].copy(), policy_list[1].copy()
    if float(wait['value']) <= translate_seconds:
        return translate_policy(policy_list[2:] + [state, wait], translate_seconds - float(wait['value']))
    new_state = state
    new_wait = wait.copy()
    wait['value'] = str(round(float(wait['value']) - translate_seconds, 5))
    new_wait['value'] = str(translate_seconds)
    return [state, wait] + policy_list[2:] + [new_state, new_wait]


sim = lgsvl.Simulator()

sim.remote.command('simulator/load_local_scene', {'scene': 'TrafficLight', 'seed': 28})

for t in sim.get_controllables():
    t.control(translate_policy(t.default_control_policy, 14.99))
st = sim.get_controllables()[0]
t = sim.get_controllables()[2]
sim.remote.command('signals/start_record', {'signals': [t.uid, st.uid]})

ego = sim.add_agent('Sedan', lgsvl.AgentType.NPC, color=lgsvl.Vector(0, 0, 1))
front = sim.add_agent('Sedan', lgsvl.AgentType.NPC, color=lgsvl.Vector(0, 1, 0))

ego_route = ['lane_3', 'lane_13', 'lane_6']

sim.remote.command('vehicle/lane_state/set', {
    'uid': ego.uid,
    'lane_id': ego_route[0],
    'offset': -experiment_args.xe - experiment_args.ve * 0.01,
})

sim.remote.command('vehicle/lane_state/set', {
    'uid': front.uid,
    'lane_id': ego_route[-1],
    'offset': experiment_args.xf + front.bounding_box.size.z + 4,
})


sim.remote.command("vehicle/follow_itinerary", {
    'follow': True,
    'uid': ego.uid,
    'itinerary': ego_route,
    'end_offset': 900,
    'initial_speed': experiment_args.ve,
    'speed_limit': Context.vl_mps
})

sim.remote.command("vehicle/follow_itinerary", {
    'follow': True,
    'uid': front.uid,
    'itinerary': ego_route[-1:],
    'end_offset': experiment_args.xf + 100,
    'initial_speed': 0,
    'speed_limit': 1e-5
})
while True:
    sim.run(3)
    if ego.state.velocity.magnitude() < 1e-5:
        sim.run(3)
        if ego.state.velocity.magnitude() < 1e-5:
            break

traffic_light_record = sim.remote.command('signals/get_record')
self_traffic_light_trace = traffic_light_record[t.uid]
side_traffic_light_trace = traffic_light_record[st.uid]
ego_trace = sim.remote.command("vehicle/get_state_record", {"uid": ego.uid})

yellow_tick = None
red_tick = None
side_tick = None

def finish():
    info = {
        've': experiment_args.ve,
        'xe': experiment_args.xe,
        'xf': experiment_args.xf,
        'ego_trace': ego_trace,
        'self_traffic_light_trace': self_traffic_light_trace,
        'side_traffic_light_trace': side_traffic_light_trace
    }

    print(json.dumps(info))
    exit()
    
finish()