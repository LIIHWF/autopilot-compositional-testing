import lgsvl
import loguru
import json
from src.simulator.adapter.argument import make_arg_parser
from src.simulator.context import Context


experiment_args = make_arg_parser('lgsvl', 'crossing_with_yield_signs').parse_args()

assert experiment_args.xa is not None

sim = lgsvl.Simulator()

sim.remote.command('simulator/load_local_scene', {'scene': 'Crossing', 'seed': 28})


ego = sim.add_agent('Sedan', lgsvl.AgentType.NPC, color=lgsvl.Vector(0, 0, 1))
arriving = sim.add_agent('Sedan', lgsvl.AgentType.NPC, color=lgsvl.Vector(1, 0, 0))
front = sim.add_agent('Sedan', lgsvl.AgentType.NPC, color=lgsvl.Vector(0, 1, 0))

ego_route = ['lane_3', 'lane_13', 'lane_6']
arriving_route = ['lane_4', 'lane_14', 'lane_0']

collision = False


sim.remote.command('vehicle/lane_state/set', {
    'uid': ego.uid,
    'lane_id': ego_route[0],
    'offset': -experiment_args.xe,
})

sim.remote.command('vehicle/lane_state/set', {
    'uid': arriving.uid,
    'lane_id': arriving_route[0],
    'offset': -max(experiment_args.xa, 0.05),
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
    'uid': arriving.uid,
    'itinerary': arriving_route,
    'end_offset': 900,
    'initial_speed': Context.vl_mps,
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

def on_collision(agent1, agent2, contact):
    global result
    result = 'collision'
    finish()

ego.on_collision(on_collision)


result = 'unknown'


def finish():
    ego_trace = sim.remote.command("vehicle/get_state_record", {"uid": ego.uid})
    arriving_trace = sim.remote.command("vehicle/get_state_record", {"uid": arriving.uid})
    front_trace = sim.remote.command("vehicle/get_state_record", {"uid": front.uid})
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


sim.run(30)
finish()
