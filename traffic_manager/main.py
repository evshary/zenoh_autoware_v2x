import argparse
import json
import logging
import time

import carla
import zenoh
from zenoh import Reliability, Sample

log_level = logging.INFO
logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

# --- Command line argument parsing --- --- --- --- --- ---
parser = argparse.ArgumentParser(prog='z_sub', description='zenoh sub example')
parser.add_argument('--mode', '-m', dest='mode', choices=['peer', 'client'], type=str, help='The zenoh session mode.')
parser.add_argument('--connect', '-e', dest='connect', metavar='ENDPOINT', action='append', type=str, help='Endpoints to connect to.')
parser.add_argument('--listen', '-l', dest='listen', metavar='ENDPOINT', action='append', type=str, help='Endpoints to listen on.')
parser.add_argument('--key', '-k', dest='key', default='vehicle/**/pose', type=str, help='The key expression to subscribe to.')
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

traffic_lights = None

lane_to_light = {
    25355: 33,
    21186: 34,
    25648: 35,  # A
    21093: 18,
    26359: 19,
    21828: 20,  # B
    21507: 16,
    24283: 17,
    24578: 13,  # C
    25539: 30,
    18400: 31,
    25838: 32,  # D
    24756: 10,
    17947: 11,
    24469: 12,  # E
    25743: 27,
    22808: 28,
    26284: 29,  # F
    25202: 7,
    22355: 8,
    24667: 9,  # G
}


A = [33, 34, 35]
B = [18, 19, 20]
C = [13, 16, 17]
D = [30, 31, 32]
E = [10, 11, 12]
F = [27, 28, 29]
G = [7, 8, 9]


# Current intersection status
current_A = []
current_B = []
current_C = []
current_D = []
current_E = []
current_F = []
current_G = []

# The number of vehicles in the intersection A, B, C, D, ...
intersection_status = [current_A, current_B, current_C, current_D, current_E, current_F, current_G]

def format_status(status):
    formatted = []
    for i, intersection in enumerate(status):
        intersection_name = chr(65 + i)
        for query in intersection:
            vehicle_id, light, distance = query
            formatted.append(f"Intersection {intersection_name}: Vehicle {vehicle_id} at light {light} is {distance:.2f} meters away")
    return "\n".join(formatted)

def consensus(vehicle_id, intersection_id, light, distance):
    global intersection_status, traffic_lights

    iter = enumerate(zip(A, B, C, D, E, F, G))
    intersection = [A, B, C, D, E, F, G]

    # intersection index
    idx = 0

    for i, (a, b, c, d, e, f, g) in iter:
        try:
            idx = [a, b, c, d, e, f, g].index(light)
        except Exception as _e:
            pass

    flag = 0

    # Check the traffic light query exist or not
    # query = [vehicle_id, light_id, distance]
    for current_x in intersection_status:
        for index, query in enumerate(current_x):
            if vehicle_id in query[0]:
                flag = 1
                # New query arrived
                if light != query[1]:
                    current_x.remove([query[0], query[1], query[2]])
                    flag = 0
                else:
                    query[2] = distance  # Update distance

    # Append new traffic light query
    if flag == 0:
        intersection_status[idx].append([vehicle_id, light, distance])
    logging.info(format_status(intersection_status))

    # Sorting every queries by vehicle_id
    for current_x in intersection_status:
        current_x.sort()
        # logging.info(current_x)
        try:
            for query in current_x:
                # Apply one query that distance below 20
                if query[2] < 20:
                    for i in intersection[idx]:
                        if i != query[1]:
                            traffic_lights[i].set_state(carla.TrafficLightState.Red)
                    traffic_lights[query[1]].set_state(carla.TrafficLightState.Green)
                    break
        except Exception as _e:
            pass


def traffic_management(vehicle_id, lane_id, x, y, z):
    if lane_id != 0:
        light = lane_to_light[lane_id]

        iter = enumerate(zip(A, B, C, D, E, F, G))
        _intersection = [A, B, C, D, E, F, G]

        idx = 0

        for i, (a, b, c, d, e, f, g) in iter:
            try:
                idx = [a, b, c, d, e, f, g].index(light)
                intersection_id = chr(idx + 65)
            except Exception as _e:
                pass

        tl_location = traffic_lights[light].get_location()

        # The y-axis has a negative sign difference between Autoware topic and Carla sim.
        vehicle_location = carla.Location(x, y * -1, z)
        distance = vehicle_location.distance(tl_location)
        consensus(vehicle_id, intersection_id, light, distance)


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
        logging.info('[traffic manager] Get Carla traffic lights')

    # initiate logging
    zenoh.try_init_log_from_env()

    logging.info('Opening session...')
    session = zenoh.open(conf)

    logging.info("Declaring Subscriber on '{}'...".format(key))

    logging.info('Connection Successed')

    def listener(sample: Sample):
        payload = json.loads(sample.payload.deserialize(str))
        lane_id = int(payload['lane_id'])
        position = payload['position']
        pos_x = float(position['x'])
        pos_y = float(position['y'])
        pos_z = float(position['z'])
        vehicle_id = str(sample.key_expr).split('/')[1]

        traffic_management(vehicle_id, lane_id, pos_x, pos_y, pos_z)

    session.declare_subscriber(key, listener, reliability=Reliability.RELIABLE)
    while True:
        time.sleep(1)


if __name__ == '__main__':
    main(args)
