import lgsvl
import loguru
from src.simulator.adapter.argument import make_arg_parser
from src.simulator.context import Context
import json

experiment_args = make_arg_parser('lgsvl', 'merging').parse_args()

assert experiment_args.ve is not None
assert experiment_args.xe is not None
assert experiment_args.xf is not None
assert experiment_args.xa is not None

sim = lgsvl.Simulator()

sim.remote.command('simulator/load_local_scene',
                   {'scene': 'Merging', 'seed': 28})

ego = sim.add_agent('Sedan', lgsvl.AgentType.NPC, color=lgsvl.Vector(0, 0, 1))
arriving = sim.add_agent('Sedan', lgsvl.AgentType.NPC,
                         color=lgsvl.Vector(1, 0, 0))
front = sim.add_agent('Sedan', lgsvl.AgentType.NPC,
                      color=lgsvl.Vector(0, 1, 0))

ego_route = ['lane_2', 'lane_4', 'lane_1']
arriving_route = ['lane_0', 'lane_3', 'lane_1']

collision = False

ego_merging_offset = 1.5


sim.remote.command('vehicle/lane_state/set', {
    'uid': ego.uid,
    'lane_id': ego_route[0] if experiment_args.xe > ego_merging_offset else ego_route[1],
    'offset': -experiment_args.xe + ego_merging_offset,
})

sim.remote.command('vehicle/lane_state/set', {
    'uid': arriving.uid,
    'lane_id': arriving_route[0],
    'offset': -max(experiment_args.xa - 5, 0.1),
})

XF = experiment_args.xf + front.bounding_box.size.z + 4.5 + ego_merging_offset

if XF > sim.remote.command('map/get_lanes')['lane_4']['length']:
    f_lane_id = ego_route[-1]
    XF = XF - sim.remote.command('map/get_lanes')['lane_4']['length']
else:
    f_lane_id = ego_route[-2]

sim.remote.command('vehicle/lane_state/set', {
    'uid': front.uid,
    'lane_id': f_lane_id,
    'offset': XF,
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
    'uid': arriving.uid,
    'itinerary': arriving_route,
    'end_offset': 900,
    'initial_speed': Context.vl_mps,
    'speed_limit': Context.vl_mps
})

sim.remote.command("vehicle/follow_itinerary", {
    'follow': True,
    'uid': front.uid,
    'itinerary': [f_lane_id],
    'end_offset': 0,
    'initial_speed': 0,
    'speed_limit': 1e-5
})


def on_collision(agent1, agent2, contact):
    global result
    result = 'collision'
    finish()


ego.on_collision(on_collision)


result = 'unknown'


def finish():
    ego_trace = ego_seq = sim.remote.command(
        "vehicle/get_state_record", {"uid": ego.uid})
    arriving_trace = arriving_seq = sim.remote.command(
        "vehicle/get_state_record", {"uid": arriving.uid})
    front_trace = front_seq = sim.remote.command(
        "vehicle/get_state_record", {"uid": front.uid})
    state_sequence = {
        'ego': ego_trace,
        'arriving': arriving_trace,
        'front': front_trace
    }

    info = {
        've': experiment_args.ve,
        'xe': experiment_args.xe,
        'xa': experiment_args.xa,
        'xf': experiment_args.xf,
        'state_sequence': state_sequence,
        'result': result
    }

    print(json.dumps(info))
    exit()

while True:
    sim.run(5, 100)
    loguru.logger.info(f'ego: {ego.state.speed}, arriving: {arriving.state.speed}')
    if ego.state.speed < 1e-4 and arriving.state.speed < 1e-4:
        break
finish()
