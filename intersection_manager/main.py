import argparse
import json
import logging
import time

import carla
import zenoh
import json
from zenoh import Sample

log_level = logging.INFO
logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

traffic_lights = None

# --- Command line argument parsing --- --- --- --- --- ---
parser = argparse.ArgumentParser(prog='z_queryable', description='zenoh queryable example')
parser.add_argument('--mode', '-m', dest='mode', choices=['peer', 'client'], type=str, help='The zenoh session mode.')
parser.add_argument('--connect', '-e', dest='connect', metavar='ENDPOINT', action='append', type=str, help='Endpoints to connect to.')
parser.add_argument('--listen', '-l', dest='listen', metavar='ENDPOINT', action='append', type=str, help='Endpoints to listen on.')
parser.add_argument(
    '--key',
    '-k',
    dest='key',
    default='intersection/**/traffic_light/**',
    type=str,
    help='The key expression matching queries to reply to.',
)
parser.add_argument('--value', '-v', dest='value', default='ACK', type=str, help='The value to reply to queries.')
parser.add_argument(
    '--complete',
    dest='complete',
    default=False,
    action='store_true',
    help='Declare the queryable as complete w.r.t. the key expression.',
)
parser.add_argument('--config', '-c', dest='config', metavar='FILE', type=str, help='A configuration file.')
parser.add_argument('--host', dest='host', type=str, default='localhost', help='Carla client host connection.')
parser.add_argument('--port', '-p', dest='port', type=int, default=2000, help='Carla client port number.')

args = parser.parse_args()
conf = zenoh.Config.from_file(args.config) if args.config is not None else zenoh.Config()
if args.mode is not None:
    conf.insert_json5(zenoh.config.MODE_KEY, json.dumps(args.mode))
if args.connect is not None:
    conf.insert_json5(zenoh.config.CONNECT_KEY, json.dumps(args.connect))
if args.listen is not None:
    conf.insert_json5(zenoh.config.LISTEN_KEY, json.dumps(args.listen))
key = args.key
value = args.value
complete = args.complete

id_to_index = {
    29855: 33,
    29777: 34,
    29861: 35,  # A
    29783: 18,
    29903: 19,
    29789: 20,  # B
    29795: 16,
    29819: 17,
    29825: 13,  # C
    29867: 30,
    29741: 31,
    29873: 32,  # D
    29927: 10,
    29747: 11,
    29831: 12,  # E
    29879: 27,
    29801: 28,
    29885: 29,  # F
    29921: 7,
    29807: 8,
    29837: 9,  # G    
}

A = [33, 34, 35]
B = [18, 19, 20]
C = [13, 16, 17]
D = [30, 31, 32]
E = [10, 11, 12]
F = [27, 28, 29]
G = [7, 8, 9]


def set_state(selector, new_state):
    global traffic_lights
    id = int(str(selector).split('/')[3])

    index = id_to_index[id]
    # print("index: ", index)
    iter = enumerate(zip(A, B, C, D, E, F, G))
    intersection = [A, B, C, D, E, F, G]

    idx = 0

    for i, (a, b, c, d, e, f, g) in iter:
        try:
            idx = [a, b, c, d, e, f, g].index(index)
        except Exception as _e:
            pass

    if new_state == 'green':
        for i in intersection[idx]:
            if i != index:
                traffic_lights[i].set_state(carla.TrafficLightState.Red)
        traffic_lights[index].set_state(carla.TrafficLightState.Green)
    elif new_state == 'red':
        traffic_lights[index].set_state(carla.TrafficLightState.Red)


def get_state(selector):
    global traffic_lights

    id = int(str(selector).split('/')[3])

    index = id_to_index[id]

    state = traffic_lights[int(index)].get_state()

    # logging.info(state)

    return state


def queryable_callback(query):
    global traffic_lights

    # print(f">> [Queryable ] Received Query '{query.selector}'" + (f" with value: {query.value.payload}" if query.value is not None else ""))
    if query.payload is None:
        # Get traffic light state
        state = get_state(query.selector)
        query.reply(str(query.selector), str(state))
        
    else:
        _new_state = query.payload.deserialize(str)
        # set_state(query.selector, new_state)


def main(args):
    global traffic_lights

    # create a client in the Carla simulator
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    # client.get_available_maps()
    world = client.get_world()

    # Get traffic lights
    traffic_lights = world.get_actors().filter('traffic.traffic_light')
    if traffic_lights:
        logging.info('[intersection manager] Get Carla traffic lights')

    # Set default Carla traffic lights' status
    for i in range(1, 36):
        traffic_lights[i].set_red_time(1000.0)
        traffic_lights[i].set_green_time(5.0)
        traffic_lights[i].set_yellow_time(0.0)
        traffic_lights[i].set_state(carla.TrafficLightState.Red)

    # initiate logging
    zenoh.try_init_log_from_env()

    logging.info("Opening session...")
    with zenoh.open(conf) as session:
        logging.info("Declaring Queryable on '{}'...".format(key))
        session.declare_queryable(key, queryable_callback, complete=complete)

        logging.info("Press CTRL-C to quit...")
        while True:
            try:
                time.sleep(1)
            except Exception as err:
                logging.info(err, flush=True)
                raise


if __name__ == '__main__':
    main(args)
