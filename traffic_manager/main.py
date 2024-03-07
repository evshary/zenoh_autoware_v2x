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

def main():

    global traffic_lights

    
    # create a client in the Carla simulator
    client = carla.Client('localhost', 2000)
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

        lane_id = int(sample.payload.decode('utf-8'))
        vehicle_id = str(sample.key_expr).split('/')[1]

        light = lane_to_light[lane_id]

        iter = enumerate(zip(A, B, C, D, E, F, G))
        intersection = [A, B, C, D, E, F, G]

        idx = 0

        for i, (a,b,c,d,e,f,g) in iter:
            try:
                idx = [a,b,c,d,e,f,g].index(light)
            except:
                pass

        for i in intersection[idx]:
            if i != light:
                traffic_lights[i].set_state(carla.TrafficLightState.Red)
        traffic_lights[light].set_state(carla.TrafficLightState.Green)

        logging.info(f">> [Subscriber] Received {sample.kind} ('{sample.key_expr}': '{sample.payload.decode('utf-8')}')")

    sub = session.declare_subscriber(key, listener, reliability=Reliability.RELIABLE())

    while True:
        time.sleep(1)

main()
