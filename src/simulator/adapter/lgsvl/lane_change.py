import lgsvl
import loguru
from src.simulator.adapter.argument import make_arg_parser
from src.simulator.context import Context
import json

experiment_args = make_arg_parser('lgsvl', 'lane_change').parse_args()

lane_change_offset = 10

sim = lgsvl.Simulator()

sim.remote.command('simulator/load_local_scene',
                   {'scene': 'LaneChange', 'seed': 28})

collision = False

lane_1 = 'lane_1'
lane_2 = 'lane_0'

ego = sim.add_agent('Sedan', lgsvl.AgentType.NPC, color=lgsvl.Vector(0, 0, 1))


sim.remote.command('vehicle/lane_state/set', {
    'uid': ego.uid,
    'lane_id': lane_1,
    'offset': 500,
})
ego.follow_closest_lane(True, Context.vl_mps, True)

sim.remote.command('vehicle/lane_state/set', {
    'uid': ego.uid,
    'lane_id': lane_1,
    'offset': 500,
    'initial_speed': experiment_args.ve
})
ego.change_lane(True)


obstacle = sim.add_agent('Sedan', lgsvl.AgentType.NPC,
                         color=lgsvl.Vector(1, 1, 0))
sim.remote.command('vehicle/lane_state/set', {
    'uid': obstacle.uid,
    'lane_id': lane_1,
    'offset': 500 + obstacle.bounding_box.size.z + 7 + experiment_args.xff,
})
obstacle.follow_closest_lane(True, 1e-5)


arriving = sim.add_agent('Sedan', lgsvl.AgentType.NPC,
                         color=lgsvl.Vector(1, 0, 0))
sim.remote.command('vehicle/lane_state/set', {
    'uid': arriving.uid,
    'lane_id': lane_2,
    'offset': 500 - experiment_args.xa + 10,
})

sim.remote.command("vehicle/follow_itinerary", {
    'follow': True,
    'uid': arriving.uid,
    'itinerary': [lane_2],
    'initial_speed': Context.vl_mps,
    'speed_limit': Context.vl_mps
})

front = sim.add_agent('Sedan', lgsvl.AgentType.NPC,
                      color=lgsvl.Vector(0, 1, 0))
sim.remote.command('vehicle/lane_state/set', {
    'uid': front.uid,
    'lane_id': lane_2,
    'offset': 500 + experiment_args.xf + lane_change_offset + front.bounding_box.size.z + {
        5: 2.5,
        10: 2.2,
        15: 1.5,
        20: 1.5
    }[experiment_args.ve],
})
front.follow_closest_lane(True, 1e-5)

distance_list = [0]
speed_list = [experiment_args.ve]

total_time = 0

start_dis = 0
end_dis = 0

prev_position = ego.state.position
start_position = ego.state.position


result = 'unknown'


def on_collision(agent1, agent2, contact):
    global result
    result = 'collision'
    finish()


ego.on_collision(on_collision)

start_tick = None
end_tick = None


def finish():
    
    state_sequence.append({
        'ego': {
            'transform': ego.transform.to_json(),
            'state': ego.state.to_json(),
        },
        'arriving': {
            'transform': arriving.transform.to_json(),
            'state': arriving.state.to_json()
        },
        'front': {
            'transform': front.transform.to_json(),
            'state': front.state.to_json()
        },
        'obstacle': {
            'transform': obstacle.transform.to_json(),
            'state': obstacle.state.to_json()
        }
    })


    info = {
        've': experiment_args.ve,
        'xf\'': experiment_args.xff,
        'xa': experiment_args.xa,
        'xf': experiment_args.xf,
        'state_sequence': state_sequence,
        'result': result
    }

    print(json.dumps(info))
    exit()


state_sequence = []

state_sequence.append({
        'ego': {
            'transform': ego.transform.to_json(),
            'state': ego.state.to_json(),
        },
        'arriving': {
            'transform': arriving.transform.to_json(),
            'state': arriving.state.to_json()
        },
        'front': {
            'transform': front.transform.to_json(),
            'state': front.state.to_json()
        },
        'obstacle': {
            'transform': obstacle.transform.to_json(),
            'state': obstacle.state.to_json()
        }
    })

sim.run(0.01)
while True:
    if ego.state.speed < 1e-3 and arriving.state.speed < 1e-3:
        sim.run(2, 10)
        break
    sim.run(2, 10)
    total_time += 2
    state_sequence.append({
        'ego': {
            'transform': ego.transform.to_json(),
            'state': ego.state.to_json(),
        },
        'arriving': {
            'transform': arriving.transform.to_json(),
            'state': arriving.state.to_json()
        },
        'front': {
            'transform': front.transform.to_json(),
            'state': front.state.to_json()
        },
        'obstacle': {
            'transform': obstacle.transform.to_json(),
            'state': obstacle.state.to_json()
        }
    })

    arriving_front_position = arriving.state.position
    front_front_position = front.state.position
    ego_front_position = ego.transform.position

    arriving_front_distance = front_front_position.z - arriving_front_position.z

finish()
