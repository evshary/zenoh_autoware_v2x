import asyncio
import heapq
import json
import logging
import math
import os
import queue
from enum import Enum
from functools import wraps

import zenoh

logging.basicConfig(format=' %(levelname)s: %(message)s', level=logging.INFO)


def trace(func):
    if asyncio.iscoroutinefunction(func):

        @wraps(func)
        async def wrapper(*args, **kwargs):
            self = args[0] if args else None
            section_id = getattr(self, 'section_id', '')
            prefix = f'[Traffic Manager][{section_id}]' if section_id else '[Traffic Manager]'
            logging.debug(f'{prefix} Starting async: {func.__name__}')
            result = await func(*args, **kwargs)
            logging.debug(f'{prefix} Finished async: {func.__name__}')
            return result

    else:

        @wraps(func)
        def wrapper(*args, **kwargs):
            self = args[0] if args else None
            section_id = getattr(self, 'section_id', '')
            prefix = f'[Traffic Manager][{section_id}]' if section_id else '[Traffic Manager]'
            logging.debug(f'{prefix} Starting sync: {func.__name__}')
            result = func(*args, **kwargs)
            logging.debug(f'{prefix} Finished sync: {func.__name__}')
            return result

    return wrapper


class TrafficLightControl(Enum):
    FREEZE_GREEN = 0
    FREEZE_RED = 1
    UNFREEZE = 2


class Intersection:
    def __init__(self, section_id, lane_info):
        self.section_id = section_id
        self.lane_info = lane_info
        self.vehicle_passing = False

        # key=lane_id, value=vehicle heap
        self.request_queue = {lane_id: [] for lane_id in lane_info.keys()}
        self.lane_last_vehicle = {
            lane_id: -1 for lane_id in lane_info.keys()
        }  # The record of the last car in each lane that has been added to the request_queue.
        self.distance_record = {}
        self.vehicle_push = {}  # The record has been added to the request queue for the vehicle
        self.last_pass = -1

    @trace
    def _decide_priority_lane(self, tx):
        _top_list = []
        for _, vehicle_heap in self.request_queue.items():
            if vehicle_heap:
                _top_list.append(vehicle_heap[0])

        if _top_list:
            self.vehicle_passing = True
            highest_priority = min(_top_list, key=lambda x: x[0])
            (_, specified_lane, specified_tl) = highest_priority

            # duration = (len(self.request_queue[specified_lane])-1)*3+5
            tx.put((self.section_id, specified_tl, TrafficLightControl.FREEZE_GREEN, 0))
            self.last_pass = self.lane_last_vehicle[specified_lane]
            logging.debug(f"[Traffic Manager] Last vehicle being processed on the prioritized lane: {self.last_pass}")

    @trace
    def _distance_measure(self, vehicle_id, vehicle_pos, lane_id):
        light_info = self.lane_info.get(lane_id)
        (x1, y1) = vehicle_pos
        (x2, y2) = light_info.get('light_pos')
        light_id = light_info.get('light_id')
        distance = math.hypot(x1 - x2, y1 - y2)
        if distance <= 15:
            pre_distance = self.distance_record.get(vehicle_id)
            if pre_distance is not None:
                if pre_distance > distance:
                    del self.distance_record[vehicle_id]
                    self.vehicle_push[vehicle_id] = (lane_id, light_id)
                    self.lane_last_vehicle[lane_id] = vehicle_id
                    heapq.heappush(self.request_queue[lane_id], (vehicle_id, lane_id, light_id))
                    logging.debug(f"[Traffic Manager] Vehicle {vehicle_id} added to lane {lane_id}'s heap with distance {distance}")
            else:
                self.distance_record[vehicle_id] = distance

    @trace
    async def monitor_request_queue(self, tx):
        while True:
            if self.vehicle_passing:
                await asyncio.sleep(0)  # to another section
                continue
            has_requests = any(queue for queue in self.request_queue.values())

            if has_requests:
                logging.debug('[Traffic Manager] monitor_request_queue', self.request_queue)
                self._decide_priority_lane(tx)
                await asyncio.sleep(0)  # to another section
            else:
                await asyncio.sleep(0)

    @trace
    def handle_query(self, vehicle_info):
        (vehicle_id, vehicle_pos, lane_id) = vehicle_info
        if self.vehicle_push.get(vehicle_id):
            return
        self._distance_measure(vehicle_id, vehicle_pos, lane_id)

    @trace
    def change_state(self, vehicle_id, tx):
        # maintain leaving vehicles
        try:
            processing_vehicle = self.vehicle_push.get(vehicle_id)
            if processing_vehicle is None:
                logging.debug(f"No ongoing vehicle with ID {vehicle_id} found at intersection {self.section_id}.")
                return
            lane_id, specified_tl = processing_vehicle
            self.vehicle_push.pop(vehicle_id, None)
            self.distance_record.pop(vehicle_id, None)
            for idx, (v_id, _, _) in enumerate(self.request_queue[lane_id]):
                if v_id == vehicle_id:
                    self.request_queue[lane_id][idx] = self.request_queue[lane_id][-1]
                    self.request_queue[lane_id].pop()
                    heapq.heapify(self.request_queue[lane_id])
            if vehicle_id == self.last_pass:
                logging.info(f"Vehicle [{vehicle_id}], last in lane [{lane_id}]'s previous queue, has passed intersection [{self.section_id}].")
                tx.put((self.section_id, specified_tl, TrafficLightControl.UNFREEZE, 0))
                self.vehicle_passing = False

        except Exception as _:
            logging.error("error happen", exc_info=True)


def load_map_info(file_path):
    with open(file_path, 'r') as f:
        map_info = json.load(f)

    lane_to_light = {}
    lane_to_intersection = {}
    for section_id, lanes in map_info['intersections'].items():
        section = {}
        for lane in lanes:
            lane_id = lane.get('autoware_lane')
            light_id = lane.get('autoware_traffic_light')
            tl_pos = lane.get('traffic_light_position')
            section[lane_id] = {
                'light_id': light_id,
                'light_pos': (float(tl_pos.get('x')), float(tl_pos.get('y'))),
            }
            lane_to_intersection[lane_id] = section_id
        lane_to_light[section_id] = section
    return lane_to_light, lane_to_intersection


@trace
async def _send_decision(session, tx):
    while True:
        while not tx.empty():
            result = tx.get()
            (section, traffic_light, command, delay) = result
            get_key = f'intersection/{section}/traffic_light/{traffic_light}/state'
            payload = None
            if command == TrafficLightControl.FREEZE_GREEN:
                payload = {'state': 'Green', 'duration': str(delay)}
            elif command == TrafficLightControl.UNFREEZE:
                payload = {'state': 'Unfreeze', 'duration': str(delay)}
            logging.info(f"[Traffic Manager] Sending Query '{get_key}'...payload: '{payload}'...")
            replies = session.get(get_key, payload=str(payload), timeout=5.0)
            for reply in replies:
                try:
                    logging.debug(f">> Received ('{reply.ok.key_expr}': '{reply.ok.payload.to_string()}')")
                except Exception as e:
                    logging.error(f">> Received (ERROR: '{reply.err.payload.to_string()}') - {e}")
        await asyncio.sleep(0)


@trace
async def subscriber(session, tx):
    vehicle_trace = {}

    @trace
    def listener(sample: zenoh.Sample):
        nonlocal vehicle_trace
        payload = json.loads(sample.payload.to_string())
        lane_id = int(payload.get('lane_id'))
        position = payload.get('position')
        pos_x = float(position.get('x'))
        pos_y = float(position.get('y'))
        vehicle_id = int(str(sample.key_expr).rsplit('/v')[-1])
        vehicle_info = (vehicle_id, (pos_x, pos_y), lane_id)
        curr_section = lane_to_intersection.get(lane_id)
        if curr_section is not None:
            if vehicle_trace.get(vehicle_id) is not None:
                prev_section = vehicle_trace.get(vehicle_id)
                if prev_section != curr_section:
                    intersections.get(prev_section).change_state(vehicle_id, tx)
            vehicle_trace[vehicle_id] = curr_section
            intersections.get(curr_section).handle_query(vehicle_info)

    logging.info("[Traffic Manager] Declaring Subscriber on 'vehicle/pose/**'...")

    session.declare_subscriber('vehicle/pose/**', listener)

    while True:
        await asyncio.sleep(0)


@trace
async def decision_making(session, tx):
    tasks = []
    for intersection in intersections.values():
        tasks.append(intersection.monitor_request_queue(tx))
    tasks.append(_send_decision(session, tx))
    await asyncio.gather(*tasks)


async def start_sub_and_decision_making():
    # initiate logging
    zenoh.init_log_from_env_or('error')
    logging.info('[Traffic Manager] Opening session...')
    with zenoh.open(zenoh.Config()) as session:
        tx = queue.Queue()
        await asyncio.gather(*[subscriber(session, tx), decision_making(session, tx)])


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(prog='traffic_manager')
    default_config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../map_info.json')
    parser.add_argument(
        '--map-info',
        dest='map_info',
        type=str,
        default=default_config_path,
        help='Path to the map information file.',
    )

    args = parser.parse_args()

    lane_to_light, lane_to_intersection = load_map_info(args.map_info)

    intersections = {}
    for section_id, lane_info in lane_to_light.items():
        intersections[section_id] = Intersection(section_id, lane_info)

    asyncio.run(start_sub_and_decision_making())
