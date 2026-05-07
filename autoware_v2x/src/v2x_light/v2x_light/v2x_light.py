#!/usr/bin/env python
import json
from argparse import ArgumentParser
from enum import Enum

import rclpy
import zenoh
from autoware_adapi_v1_msgs.msg import VehicleKinematics
from autoware_internal_planning_msgs.msg import PathWithLaneId
from autoware_perception_msgs.msg import TrafficLightElement, TrafficLightGroup, TrafficLightGroupArray
from rclpy.node import Node
from rclpy.qos import QoSProfile
from zenoh import Config, QueryTarget, Reliability

"""
ref link : https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_perception_msgs/msg/TrafficLight.idl

# constants for common use
uint8 UNKNOWN = 0

# constants for color
uint8 RED = 1
uint8 AMBER = 2
uint8 GREEN = 3
uint8 WHITE = 4

# constants for shape
uint8 CIRCLE = 1
uint8 LEFT_ARROW = 2
uint8 RIGHT_ARROW = 3
uint8 UP_ARROW = 4
uint8 UP_LEFT_ARROW=5
uint8 UP_RIGHT_ARROW=6
uint8 DOWN_ARROW = 7
uint8 DOWN_LEFT_ARROW = 8
uint8 DOWN_RIGHT_ARROW = 9
uint8 CROSS = 10

# constants for status
uint8 SOLID_OFF = 1
uint8 SOLID_ON = 2
uint8 FLASHING = 3

# variables
uint8 color
uint8 shape
uint8 status
float32 confidence
"""

parser = ArgumentParser()
parser.add_argument('--vehicle', '-v', type=str, default='v1', help='Vehicle ID')
parser.add_argument(
    '--map-info',
    dest='map_info',
    type=str,
    default='external/zenoh_autoware_v2x/map_info.json',
    help='Path to map_info.json (resolved relative to current working directory)',
)

args = parser.parse_args()


class Pose:
    def __init__(self, lane_id, pos_x, pos_y, pos_z):
        self.lane_id = lane_id
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.pos_z = pos_z


class Color(Enum):
    RED = 1
    AMBER = 2
    GREEN = 3
    WHITE = 4


class Shape(Enum):
    CIRCLE = 1
    LEFT_ARROW = 2
    RIGHT_ARROW = 3
    UP_ARROW = 4


class Status(Enum):
    OFF = 1
    ON = 2


confidence = 1.0

color_dict = {'Red': 1, 'Yellow': 2, 'Green': 3, 'White': 4}


def _load_map_info(path):
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


lane_id, light_id, intersection_id = _load_map_info(args.map_info)

# TODO: Please avoid using global variables here, as they may cause conflicts in multi-vehicle scenarios.
# Init. global variables about Pose
pos_lane_id = 0
pos_x = 0.0
pos_y = 0.0
pos_z = 0.0

conf = Config()
session = zenoh.open(conf)


class SignalPub(Node):
    def __init__(self, nodeName):
        super().__init__(nodeName)
        qos_profile = QoSProfile(
            reliability=1,  # RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
            history=1,  # RMW_QOS_POLICY_HISTORY_KEEP_LAST
            depth=1,
        )
        self.publication = self.create_publisher(
            TrafficLightGroupArray,
            '/perception/traffic_light_recognition/traffic_signals',
            qos_profile=qos_profile,
        )
        self.subscription = self.create_subscription(
            PathWithLaneId,
            'planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id',
            self.signal_callback,
            qos_profile=qos_profile,
        )
        self.pose_subscription = self.create_subscription(
            VehicleKinematics,
            '/api/vehicle/kinematics',
            self.pose_callback,
            QoSProfile(
                reliability=2,  # RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
                history=1,  # RMW_QOS_POLICY_HISTORY_KEEP_LAST
                depth=1,
            ),
        )

        self.pose_publisher = session.declare_publisher(f'vehicle/pose/{args.vehicle}', reliability=Reliability.RELIABLE)
        self.publish_red_signal()

    def publish_red_signal(self):
        global light_id

        self.publish_traffic_light_to_autoware(light_id, Color.RED.value)

    def publish_pose(self):
        global pos_x, pos_y, pos_z, pos_lane_id

        pose = {'lane_id': pos_lane_id, 'position': {'x': pos_x, 'y': pos_y, 'z': pos_z}}

        self.pose_publisher.put(json.dumps(pose))

    def pose_callback(self, data):
        global pos_x, pos_y, pos_z

        position = data.pose.pose.pose.position
        pos_x = position.x
        pos_y = position.y
        pos_z = position.z

        self.publish_pose()

    def signal_callback(self, data):
        global color_dict, light_id, lane_id
        global pos_lane_id

        closest_lane = []

        for d in data.points:
            if any(id in d.lane_ids for id in lane_id):
                if not closest_lane and len(d.lane_ids) == 1:
                    closest_lane.append(d.lane_ids[0])
                    pos_lane_id = closest_lane[0]
                    idx = lane_id.index(d.lane_ids[0])
                    tl_id = light_id[idx]

                    # Sync Autoware and Carla's traffic light signal
                    selector, target, color = self.query_light_status(tl_id)
                    self.publish_traffic_light_to_autoware(tl_id, color_dict.get(color, 1))

                    self.publish_pose()
                    break
            else:
                pass

    def publish_traffic_light_to_autoware(self, tl_list: list, color: int):
        global light_id
        traffic_light_group_array = TrafficLightGroupArray()
        traffic_light_group_array.stamp = self.get_clock().now().to_msg()

        for light in light_id:
            traffic_light_group = TrafficLightGroup()
            traffic_light_group.traffic_light_group_id = light
            element = TrafficLightElement()
            element.color = color
            element.shape = Shape.CIRCLE.value
            element.status = Status.ON.value
            element.confidence = 1.0
            traffic_light_group.elements.append(element)
            traffic_light_group_array.traffic_light_groups.append(traffic_light_group)

        self.publication.publish(traffic_light_group_array)
        # TODO: Use the query_light_status function here to set Carla's traffic lights.

    def query_light_status(self, tl_id):
        selector = f'intersection/{intersection_id[tl_id]}/traffic_light/' + str(tl_id) + '/state'

        target = {
            'ALL': QueryTarget.ALL,
            'BEST_MATCHING': QueryTarget.BEST_MATCHING,
            'ALL_COMPLETE': QueryTarget.ALL_COMPLETE,
        }.get('BEST_MATCHING')
        payload = None
        replies = session.get(selector, target=target, payload=None)

        for reply in replies:
            try:
                payload = reply.ok.payload.to_string()
            except Exception as _e:
                payload = reply.err.payload.to_string()

        return selector, target, payload


def main(args=args):
    rclpy.init()
    node = SignalPub('v2x_light')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
