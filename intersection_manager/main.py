import asyncio
import json
import logging
import os
import queue
from functools import wraps

import carla
import zenoh

logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

# Max distance (meters) between a map_info.json position and a CARLA traffic light actor.
TRAFFIC_LIGHT_MATCH_THRESHOLD_M = 5.0


def trace(func):
    if asyncio.iscoroutinefunction(func):

        @wraps(func)
        async def wrapper(*args, **kwargs):
            self = args[0] if args else None
            section_id = getattr(self, 'section_id', '')
            prefix = f'[Intersection Manager][{section_id}]' if section_id else '[Intersection Manager]'
            logging.debug(f'{prefix} Starting async: {func.__name__}')
            result = await func(*args, **kwargs)
            logging.debug(f'{prefix} Finished async: {func.__name__}')
            return result

    else:

        @wraps(func)
        def wrapper(*args, **kwargs):
            self = args[0] if args else None
            section_id = getattr(self, 'section_id', '')
            prefix = f'[Intersection Manager][{section_id}]' if section_id else '[Intersection Manager]'
            logging.debug(f'{prefix} Starting sync: {func.__name__}')
            result = func(*args, **kwargs)
            logging.debug(f'{prefix} Finished sync: {func.__name__}')
            return result

    return wrapper


class Intersection:
    def __init__(self, section_id, light_positions):
        self.section_id = section_id
        # {autoware_tl: (carla_x, carla_y)} — CARLA-frame positions
        self.light_positions = light_positions
        self.traffic_lights = self._match_traffic_lights()
        # self.priority_event = threading.Event()
        self.priority_event = asyncio.Event()

    def _match_traffic_lights(self):
        """Resolve each autoware traffic light to the nearest CARLA actor by position.

        CARLA actor IDs aren't stable across versions/sessions, so we match by world
        coordinates instead of relying on a hard-coded id field in map_info.json.
        """
        traffic_lights = {}
        for autoware_tl, (x, y) in self.light_positions.items():
            best = min(
                world_traffic_lights,
                key=lambda a, x=x, y=y: (a.get_location().x - x) ** 2 + (a.get_location().y - y) ** 2,
            )
            loc = best.get_location()
            dist = ((loc.x - x) ** 2 + (loc.y - y) ** 2) ** 0.5
            if dist > TRAFFIC_LIGHT_MATCH_THRESHOLD_M:
                logging.warning(
                    f'[Intersection Manager][{self.section_id}] autoware_tl {autoware_tl}: '
                    f'nearest CARLA light id={best.id} is {dist:.2f}m away '
                    f'(threshold {TRAFFIC_LIGHT_MATCH_THRESHOLD_M}m) — check map_info.json'
                )
            else:
                logging.debug(f'[Intersection Manager][{self.section_id}] autoware_tl {autoware_tl} -> CARLA id={best.id} (dist={dist:.2f}m)')
            traffic_lights[autoware_tl] = best
        return traffic_lights

    def _set_traffic_light_state(self, green_id):
        """Set traffic light states: one green, others red."""
        for autoware_tl, traffic_light in self.traffic_lights.items():
            if autoware_tl == green_id:
                traffic_light.set_state(carla.TrafficLightState.Green)
                logging.debug(f'[Intersection Manager][{self.section_id}] Traffic light {autoware_tl} set to GREEN')
            else:
                traffic_light.set_state(carla.TrafficLightState.Red)
                logging.debug(f'[Intersection Manager][{self.section_id}] Traffic light {autoware_tl} set to RED')

    @trace
    async def auto_changing_state(self):
        """Automatically change signals periodically for Intersection."""
        traffic_light_ids = list(self.traffic_lights.keys())
        while True:
            for green_id in traffic_light_ids:
                while self.priority_event.is_set():
                    logging.debug((f'[Intersection Manager][{self.section_id}] Priority active, pausing auto mode.'))
                    await asyncio.sleep(0)

                logging.debug(f'[Intersection Manager][{self.section_id}] Under auto mode.')
                self._set_traffic_light_state(green_id)
                await asyncio.sleep(5)

    @trace
    def get_state(self, autoware_tl):
        return self.traffic_lights[autoware_tl].get_state()

    @trace
    async def handle_priority(self, autoware_tl, duration):
        self.priority_event.set()
        logging.info(f'[Intersection Manager][{self.section_id}] Priority active, priority event is set.')
        for tl_id, traffic_light in self.traffic_lights.items():
            if tl_id != autoware_tl:
                traffic_light.set_state(carla.TrafficLightState.Red)
        await asyncio.sleep(1)
        self.traffic_lights[autoware_tl].set_state(carla.TrafficLightState.Green)
        if duration != 0:
            await asyncio.sleep(duration)
            self.priority_event.clear()

    def event_clear(self):
        logging.debug(f'[Intersection Manager][{self.section_id}] Priority event clear.')
        logging.info(f'[Intersection Manager][{self.section_id}] Back to auto mode.')
        self.priority_event.clear()


def load_map_info(file_path):
    """Load {section_id: {autoware_tl: (carla_x, carla_y)}} from map_info.json.

    map_info.json stores positions in Autoware coordinates (Autoware_y = -CARLA_y),
    so we negate y here to keep the rest of the code in CARLA's frame.
    """
    with open(file_path, 'r') as f:
        map_info = json.load(f)
    autoware_to_position = {}
    for section_id, lanes in map_info['intersections'].items():
        autoware_to_position[section_id] = {
            int(lane['autoware_traffic_light']): (
                float(lane['traffic_light_position']['x']),
                -float(lane['traffic_light_position']['y']),
            )
            for lane in lanes
        }
    return autoware_to_position


@trace
async def event_handler(event_queue):
    while True:
        while not event_queue.empty():
            event = event_queue.get()
            (section, tl_state, autoware_tl, duration) = event
            if tl_state == 'Green':
                asyncio.create_task(intersections[section].handle_priority(autoware_tl, duration))
            elif tl_state == 'Unfreeze':
                intersections[section].event_clear()
            else:
                raise Exception("The state to set is unrecognizable.'")
        await asyncio.sleep(0)


@trace
async def normal_operation():
    tasks = []
    for intersection in intersections.values():
        tasks.append(intersection.auto_changing_state())
    await asyncio.gather(*tasks)


@trace
async def start_queryable(event_queue):
    # initiate logging
    zenoh.init_log_from_env_or('error')

    logging.info('[Intersection Manager] Opening session...')

    with zenoh.open(zenoh.Config()) as session:
        queryable_key = 'intersection/**/traffic_light/**'

        @trace
        def queryable_callback(query):
            nonlocal event_queue
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
                    duration = int(payload['duration'])

                    event_queue.put((section, tl_state, autoware_tl, duration))

                except Exception as _:
                    raise
            else:
                state = intersections[section].get_state(autoware_tl)
                query.reply(str(query.selector), str(state))

        logging.info(f"[Intersection Manager] Declaring Queryable on '{queryable_key}'...")
        session.declare_queryable(queryable_key, queryable_callback)

        while True:
            await asyncio.sleep(0)


async def main():
    event_queue = queue.Queue()
    await asyncio.gather(*[normal_operation(), start_queryable(event_queue), event_handler(event_queue)])


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
    max_retries = 5
    for _ in range(max_retries):
        world_traffic_lights = client.get_world().get_actors().filter('traffic.traffic_light')
        if world_traffic_lights:
            break
        logging.warning('[Intersection Manager] Failed to get the actors from the world, retrying...')
    else:
        logging.error('[Intersection Manager] Unable to retrieve traffic lights after multiple attempts. Exiting...')
        exit(1)

    fix_red_light_to_all()

    intersections = {}
    for section_id, light_positions in map_info.items():
        intersections[section_id] = Intersection(section_id, light_positions)

    asyncio.run(main())
