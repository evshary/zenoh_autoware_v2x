import asyncio
import json
import logging
import os
import threading
import time
from functools import wraps

import carla
import zenoh

logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)


def trace(func):
    if asyncio.iscoroutinefunction(func):

        @wraps(func)
        async def wrapper(*args, **kwargs):
            self = args[0] if args else None
            section_id = getattr(self, 'section_id', '')
            prefix = f'[Intersection Manager][{section_id}]' if section_id else '[Traffic Manager]'
            logging.debug(f'{prefix} Starting async: {func.__name__}')
            result = await func(*args, **kwargs)
            logging.debug(f'{prefix} Finished async: {func.__name__}')
            return result

    else:

        @wraps(func)
        def wrapper(*args, **kwargs):
            self = args[0] if args else None
            section_id = getattr(self, 'section_id', '')
            prefix = f'[Intersection Manager][{section_id}]' if section_id else '[Traffic Manager]'
            logging.debug(f'{prefix} Starting sync: {func.__name__}')
            result = func(*args, **kwargs)
            logging.debug(f'{prefix} Finished sync: {func.__name__}')
            return result

    return wrapper


class Intersection:
    def __init__(self, section_id, light_mapping):
        self.section_id = section_id
        self.light_mapping = light_mapping
        self.traffic_lights = self._filter_traffic_lights()
        self.priority_event = threading.Event()

    def _filter_traffic_lights(self):
        """Retrieve specific Carla traffic lights based on IDs."""
        traffic_lights = {}
        for actor in world_traffic_lights:
            if actor.id in self.light_mapping.values():
                traffic_lights[actor.id] = actor
        for key, carla_item in traffic_lights.items():
            logging.debug(f'[Intersection Manager] Traffic Light ID {key} is getted.')
        return traffic_lights

    def _set_traffic_light_state(self, green_id):
        """Set traffic light states: one green, others red."""
        for id, traffic_light in self.traffic_lights.items():
            if id == green_id:
                traffic_light.set_state(carla.TrafficLightState.Green)
                logging.debug(f'[Intersection Manager][{self.section_id}] Traffic light {id} set to GREEN')
            else:
                traffic_light.set_state(carla.TrafficLightState.Red)
                logging.debug(f'[Intersection Manager][{self.section_id}] Traffic light {id} set to RED')

    @trace
    async def auto_changing_state(self):
        """Automatically change signals periodically for Intersection."""
        green_cycle = list(self.traffic_lights.keys())
        while True:
            for green_id in green_cycle:
                while self.priority_event.is_set():
                    logging.debug((f'[Intersection Manager][{self.section_id}] Priority active, pausing auto mode.'))
                    await asyncio.sleep(0)

                logging.debug(f'[Intersection Manager][{self.section_id}] Under auto mode.')
                self._set_traffic_light_state(green_id)
                await asyncio.sleep(5)

    @trace
    def get_state(self, autoware_tl):
        carla_tl = self.light_mapping[autoware_tl]
        return self.traffic_lights[carla_tl].get_state()

    @trace
    def handle_priority(self, autoware_tl, duration):
        self.priority_event.set()
        logging.info(f'[Intersection Manager][{self.section_id}] Priority active, priority event is set.')
        carla_tl = self.light_mapping[autoware_tl]
        for id, traffic_light in self.traffic_lights.items():
            if id != carla_tl:
                traffic_light.set_state(carla.TrafficLightState.Red)
        time.sleep(1)
        self.traffic_lights[carla_tl].set_state(carla.TrafficLightState.Green)
        time.sleep(duration)

        self.priority_event.clear()
        logging.info(f'[Intersection Manager][{self.section_id}] Back to auto mode.')


def load_map_info(file_path):
    with open(file_path, 'r') as f:
        map_info = json.load(f)
    autoware_to_carla_tl = {}
    for section_id, lanes in map_info['intersections'].items():
        autoware_to_carla_tl[section_id] = {int(lane['autoware_traffic_light']): int(lane['carla_traffic_light']) for lane in lanes}
    return autoware_to_carla_tl


@trace
async def normal_operation():
    tasks = []
    for intersection in intersections.values():
        tasks.append(intersection.auto_changing_state())
    await asyncio.gather(*tasks)


@trace
async def start_queryable():
    # initiate logging
    zenoh.init_log_from_env_or('error')

    logging.info('[Intersection Manager] Opening session...')

    with zenoh.open(zenoh.Config()) as session:
        queryable_key = 'intersection/**/traffic_light/**'

        @trace
        def queryable_callback(query):
            if query.payload is not None:
                logging.info(
                    f"[Intersection Manager][Queryable ] Received Query '{query.selector}'" + (f' with payload: {query.payload.to_string()}')
                )
            else:
                logging.debug(f"[Intersection Manager][Queryable ] Received Query '{query.selector}'with no payload")
            selector_part = str(query.selector).split('/')
            section = selector_part[1]
            autoware_tl = int(selector_part[3])
            if query.payload is not None:
                query.reply(str(query.selector), str(query.payload))
                payload = eval(query.payload.to_string())
                # For debugging
                try:
                    tl_state = payload['state']
                    if tl_state != 'Green':
                        raise Exception("state to set is not 'Green'")
                    duration = int(payload['duration'])

                    t = threading.Thread(
                        target=intersections[section].handle_priority,
                        args=(
                            autoware_tl,
                            duration,
                        ),
                    )
                    t.start()

                except Exception as err:
                    print(err, flush=True)
                    raise
            else:
                state = intersections[section].get_state(autoware_tl)
                query.reply(str(query.selector), str(state))

        logging.info(f"[Intersection Manager] Declaring Queryable on '{queryable_key}'...")
        session.declare_queryable(queryable_key, queryable_callback)

        while True:
            await asyncio.sleep(0)


async def main():
    await asyncio.gather(*[normal_operation(), start_queryable()])


def fix_red_light_to_all():
    for traffic_light in world_traffic_lights:
        traffic_light.set_state(carla.TrafficLightState.Red)
        traffic_light.freeze(True)


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(prog='intersection_manager')

    parser.add_argument(
        '--host',
        dest='carla_host',
        type=str,
        default='localhost',
        help='Carla simulation host.',
    )
    parser.add_argument(
        '--port',
        '-p',
        dest='carla_port',
        type=int,
        default=2000,
        help='Carla simulation port number.',
    )

    default_config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../map_info.json')
    parser.add_argument(
        '--map-info',
        dest='map_info',
        type=str,
        default=default_config_path,
        help='Path to the map information file.',
    )

    args = parser.parse_args()

    map_info = load_map_info(args.map_info)

    client = carla.Client(args.carla_host, args.carla_port)
    client.set_timeout(10.0)  # seconds
    # THIS IS FOR　TESTING **Do not load the world again when the bridge is running.**
    # client.load_world('Town01')
    world_traffic_lights = client.get_world().get_actors().filter('traffic.traffic_light')
    while not world_traffic_lights:
        logging.warning('[Intersection Manager] Failed to get the actors from the world, retrying...')
        world_traffic_lights = client.get_world().get_actors().filter('traffic.traffic_light')
    logging.info('[Intersection Manager] Successfully retrieved the actors from the world.')
    fix_red_light_to_all()

    intersections = {}
    for section_id, light_mapping in map_info.items():
        intersections[section_id] = Intersection(section_id, light_mapping)

    asyncio.run(main())
