import time
import argparse
import json
import zenoh
from zenoh import Reliability, Sample
import carla
import logging

log_level = logging.INFO
logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

# --- Command line argument parsing --- --- --- --- --- ---
parser = argparse.ArgumentParser(
    prog='z_sub',
    description='zenoh sub example')
parser.add_argument('--mode', '-m', dest='mode',
                    choices=['peer', 'client'],
                    type=str,
                    help='The zenoh session mode.')
parser.add_argument('--connect', '-e', dest='connect',
                    metavar='ENDPOINT',
                    action='append',
                    type=str,
                    help='Endpoints to connect to.')
parser.add_argument('--listen', '-l', dest='listen',
                    metavar='ENDPOINT',
                    action='append',
                    type=str,
                    help='Endpoints to listen on.')
parser.add_argument('--key', '-k', dest='key',
                    default='vehicle/**/pose',
                    type=str,
                    help='The key expression to subscribe to.')
parser.add_argument('--config', '-c', dest='config',
                    metavar='FILE',
                    type=str,
                    help='A configuration file.')
parser.add_argument('--host', dest='host',
                    type=str,
                    default='localhost',
                    help='Carla client host connection.')
parser.add_argument('--port', '-p', dest='port',
                    type=int,
                    default=2000,
                    help='Carla client port number.')

args = parser.parse_args()
conf = zenoh.Config.from_file(
    args.config) if args.config is not None else zenoh.Config()
if args.mode is not None:
    conf.insert_json5(zenoh.config.MODE_KEY, json.dumps(args.mode))
if args.connect is not None:
    conf.insert_json5(zenoh.config.CONNECT_KEY, json.dumps(args.connect))
if args.listen is not None:
    conf.insert_json5(zenoh.config.LISTEN_KEY, json.dumps(args.listen))
key = args.key

traffic_lights = None

lane_to_light = {
    1549: 33, 1136: 34, 1556: 35, # A
    1143: 18, 1605: 19, 1150: 20, # B
    1157: 16, 1419: 17, 1426: 13, # C
    1563: 30, 1064: 31, 1570: 32, # D
    1440: 10, 1071: 11, 1433: 12, # E
    1577: 27, 1234: 28, 1584: 29, # F
    1454:  7, 1241:  8, 1447:  9  # G
}


A = [33, 34, 35]
B = [18, 19, 20]
C = [13, 16, 17]
D = [30, 31, 32]
E = [10, 11, 12]
F = [27, 28, 29]
G = [ 7,  8,  9]


def traffic_management(vehicle_id, lane_id, x, y, z):
    if lane_id != 0:
        light = lane_to_light[lane_id]

        iter = enumerate(zip(A, B, C, D, E, F, G))
        intersection = [A, B, C, D, E, F, G]

        idx = 0

        for i, (a,b,c,d,e,f,g) in iter:
            try:
                idx = [a,b,c,d,e,f,g].index(light)
                intersection_id = chr(idx + 65)
            except:
                pass

        tl_location = traffic_lights[light].get_location()

        # The y-axis has a negative sign difference between Autoware topic and Carla sim.
        vehicle_location = carla.Location(x, y * -1, z)

        distance = vehicle_location.distance(tl_location)

        logging.info(f'The vehicle {vehicle_id} is approaching intersection {intersection_id} and will arrive in {distance:.2f} meters.')

        if distance < 15.0:
            for i in intersection[idx]:
                if i != light:
                    traffic_lights[i].set_state(carla.TrafficLightState.Red)
            traffic_lights[light].set_state(carla.TrafficLightState.Green)

def main(args):

    global traffic_lights

    
    # create a client in the Carla simulator
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    # client.get_available_maps()
    world = client.get_world()


    # Get traffic lights
    traffic_lights = world.get_actors().filter("traffic.traffic_light")
    if traffic_lights:
        logging.info(f'[traffic manager] Get Carla traffic lights')

    # initiate logging
    zenoh.init_logger()

    print("Opening session...")
    session = zenoh.open(conf)

    print("Declaring Subscriber on '{}'...".format(key))

    logging.info("Connection Successed")


    def listener(sample: Sample):
        payload = json.loads(sample.payload.decode('utf-8'))

        lane_id = int(payload['lane_id'])
        position = payload['position']
        pos_x = float(position['x'])
        pos_y = float(position['y'])
        pos_z = float(position['z'])

        vehicle_id = str(sample.key_expr).split('/')[1]

        traffic_management(vehicle_id, lane_id, pos_x, pos_y, pos_z)

    sub = session.declare_subscriber(key, listener, reliability=Reliability.RELIABLE())

    while True:
        time.sleep(1)

if __name__ == '__main__':
    main(args)
