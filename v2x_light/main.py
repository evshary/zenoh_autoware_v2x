#!/usr/bin/env python
import json
import logging
import os
import time
from argparse import ArgumentParser

import zenoh
from zenoh import QueryTarget, Reliability
from zenoh_ros_type.autoware_adapi_msgs import VehicleKinematics
from zenoh_ros_type.autoware_internal_msgs import PathWithLaneId
from zenoh_ros_type.autoware_msgs.autoware_perception_msgs import (
    TrafficLightElement,
    TrafficLightGroup,
    TrafficLightGroupArray,
)
from zenoh_ros_type.rcl_interfaces import Time
from zenoh_ros_type.rmw_zenoh import Attachment

logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

GET_PATH_WITH_LANE_ID_KEY_EXPR = '/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id'
GET_VEHICLE_KINEMATICS_KEY_EXPR = '/api/vehicle/kinematics'
SET_TRAFFIC_SIGNALS_KEY_EXPR = '/perception/traffic_light_recognition/traffic_signals'

# intersection_manager replies with str(carla.TrafficLightState); map to TrafficLightElement.COLOR.
CARLA_TO_COLOR = {
    'Red': TrafficLightElement.COLOR.RED.value,
    'Yellow': TrafficLightElement.COLOR.AMBER.value,
    'Green': TrafficLightElement.COLOR.GREEN.value,
    'Off': TrafficLightElement.COLOR.UNKNOWN.value,
    'Unknown': TrafficLightElement.COLOR.UNKNOWN.value,
}


def load_map_info(path):
    """Build (lane_id, light_id, intersection_id) from map_info.json.

    lane_id[i] corresponds to light_id[i]; intersection_id maps tl_id -> section letter.
    """
    with open(path) as f:
        data = json.load(f)
    lanes, lights, sections = [], [], {}
    for section_id, entries in data['intersections'].items():
        for entry in entries:
            lanes.append(int(entry['autoware_lane']))
            lights.append(int(entry['autoware_traffic_light']))
            sections[int(entry['autoware_traffic_light'])] = section_id
    return lanes, lights, sections


class SignalPub:
    def __init__(self, session, scope, lane_id, light_id, intersection_id, use_bridge_ros2dds=True):
        self.session = session
        self.scope = scope
        self.lane_id = lane_id
        self.light_id = light_id
        self.intersection_id = intersection_id
        self.use_bridge_ros2dds = use_bridge_ros2dds

        self.prefix = scope if use_bridge_ros2dds else scope + '/*'
        self.postfix = '' if use_bridge_ros2dds else '/**'

        # Pose state
        self.pos_lane_id = 0
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0

        # From Autoware
        self.subscriber_path = self.session.declare_subscriber(
            self.prefix + GET_PATH_WITH_LANE_ID_KEY_EXPR + self.postfix,
            self.signal_callback,
        )
        self.subscriber_kinematics = self.session.declare_subscriber(
            self.prefix + GET_VEHICLE_KINEMATICS_KEY_EXPR + self.postfix,
            self.pose_callback,
        )

        # To Autoware
        self.publisher_signal = self.session.declare_publisher(self.prefix + SET_TRAFFIC_SIGNALS_KEY_EXPR + self.postfix)

        if not use_bridge_ros2dds:
            self.attachment_signal = Attachment()

        # To bridge
        self.publisher_pose = self.session.declare_publisher(
            f'vehicle/pose/{scope}',
            reliability=Reliability.RELIABLE,
        )

        # Send initial all-red signal so Autoware sees a defined state at startup
        self.publish_red_signal()

    def publish_red_signal(self):
        self.publish_traffic_light_to_autoware(TrafficLightElement.COLOR.RED.value)
        # TODO: Use the query_light_status function here to set Carla's traffic lights.

    def publish_pose(self):
        pose = {
            'lane_id': self.pos_lane_id,
            'position': {'x': self.pos_x, 'y': self.pos_y, 'z': self.pos_z},
        }
        self.publisher_pose.put(json.dumps(pose))

    def pose_callback(self, sample: zenoh.Sample):
        kinematics = VehicleKinematics.deserialize(sample.payload.to_bytes())
        position = kinematics.pose.pose.pose.position
        self.pos_x = position.x
        self.pos_y = position.y
        self.pos_z = position.z
        self.publish_pose()

    def signal_callback(self, sample: zenoh.Sample):
        path = PathWithLaneId.deserialize(sample.payload.to_bytes())

        for point in path.points:
            if len(point.lane_ids) != 1:
                continue
            lid = point.lane_ids[0]
            if lid not in self.lane_id:
                continue
            self.pos_lane_id = lid
            tl_id = self.light_id[self.lane_id.index(lid)]

            # Sync Autoware and Carla's traffic light signal
            carla_state = self.query_light_status(tl_id)
            color = CARLA_TO_COLOR.get(carla_state, TrafficLightElement.COLOR.UNKNOWN.value)
            self.publish_traffic_light_to_autoware(color)
            self.publish_pose()
            break

    def publish_traffic_light_to_autoware(self, color: int):
        now = time.time()
        stamp = Time(sec=int(now), nanosec=int((now - int(now)) * 1e9))
        groups = []
        for light in self.light_id:
            groups.append(
                TrafficLightGroup(
                    traffic_light_group_id=light,
                    elements=[
                        TrafficLightElement(
                            color=color,
                            shape=TrafficLightElement.SHAPE.CIRCLE.value,
                            status=TrafficLightElement.STATUS.SOLID_ON.value,
                            confidence=1.0,
                        )
                    ],
                    predictions=[],
                )
            )
        msg = TrafficLightGroupArray(stamp=stamp, traffic_light_groups=groups)
        self.publisher_signal.put(
            msg.serialize(),
            attachment=None if self.use_bridge_ros2dds else self.attachment_signal.serialize(),
        )

    def query_light_status(self, tl_id):
        selector = f'intersection/{self.intersection_id[tl_id]}/traffic_light/{tl_id}/state'
        replies = self.session.get(selector, target=QueryTarget.BEST_MATCHING, payload=None)
        for reply in replies:
            try:
                return reply.ok.payload.to_string()
            except Exception:
                logging.warning(f'[V2X Light] query reply error: {reply.err.payload.to_string()}')
        return None


def main():
    parser = ArgumentParser(prog='v2x_light')
    parser.add_argument('--vehicle', '-v', type=str, default='v1', help='Vehicle ID (used as Zenoh scope)')
    parser.add_argument(
        '--rmw_zenoh',
        dest='use_rmw_zenoh',
        action='store_true',
        help='Talk to Autoware via rmw_zenoh (default: zenoh-bridge-ros2dds)',
    )
    default_config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../map_info.json')
    parser.add_argument(
        '--map-info',
        dest='map_info',
        type=str,
        default=default_config_path,
        help='Path to map_info.json',
    )
    parser.add_argument(
        '--connect',
        '-e',
        dest='connect',
        metavar='ENDPOINT',
        action='append',
        type=str,
        help='Endpoint to connect to.',
    )
    args = parser.parse_args()

    lane_id, light_id, intersection_id = load_map_info(args.map_info)

    zenoh.init_log_from_env_or('error')

    config = zenoh.Config()
    if args.connect:
        config.insert_json5('connect/endpoints', json.dumps(args.connect))

    logging.info('[V2X Light] Opening session...')
    with zenoh.open(config) as session:
        SignalPub(
            session,
            args.vehicle,
            lane_id,
            light_id,
            intersection_id,
            use_bridge_ros2dds=not args.use_rmw_zenoh,
        )
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
