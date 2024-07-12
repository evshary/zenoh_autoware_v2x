#!/usr/bin/env python
from argparse import ArgumentParser
from enum import Enum

import rclpy
import zenoh
from autoware_adapi_v1_msgs.msg import VehicleKinematics
from autoware_auto_perception_msgs.msg import TrafficLight, TrafficSignal, TrafficSignalArray
from autoware_auto_planning_msgs.msg import PathWithLaneId
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import QoSProfile
from zenoh import QueryTarget

"""
ref link : https://github.com/tier4/autoware_auto_msgs/blob/tier4/main/autoware_auto_perception_msgs/msg/TrafficLight.idl

module autoware_auto_perception_msgs {
  module msg {
    module TrafficLight_Constants {
      // constants for color
      const uint8 RED = 1;
      const uint8 AMBER = 2;
      const uint8 GREEN = 3;
      const uint8 WHITE = 4;

      // constants for shape
      const uint8 CIRCLE = 5;
      const uint8 LEFT_ARROW = 6;
      const uint8 RIGHT_ARROW = 7;
      const uint8 UP_ARROW = 8;
      const uint8 UP_LEFT_ARROW = 9;
      const uint8 UP_RIGHT_ARROW = 10;
      const uint8 DOWN_ARROW = 11;
      const uint8 DOWN_LEFT_ARROW = 12;
      const uint8 DOWN_RIGHT_ARROW = 13;
      const uint8 CROSS = 14;

      // constants for status
      const uint8 SOLID_OFF = 15;
      const uint8 SOLID_ON = 16;
      const uint8 FLASHING = 17;

      // constants for common use
      const uint8 UNKNOWN = 18;
    };
    struct TrafficLight {
      @default (value=0)
      uint8 color;

      @default (value=0)
      uint8 shape;

      @default (value=0)
      uint8 status;

      @default (value=0.0)
      float confidence;
    };
  };
};
"""

parser = ArgumentParser()
parser.add_argument('--vehicle', '-v', type=str, default='v1', help='Vehicle ID')

args = parser.parse_args()


class Pose:
    def __init__(self, lane_id, pos_x, pos_y, pos_z):
        self.lane_id = lane_id
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.pos_z = pos_z


class Shape(Enum):
    CIRCLE = 5
    LEFT_ARROW = 6
    RIGHT_ARROW = 7
    UP_ARROW = 8


class Status(Enum):
    OFF = 15
    ON = 16


confidence = 1.0

color_dict = {'Red': 1, 'Yellow': 2, 'Green': 3, 'White': 4}

lane_id = [
    1549,
    1136,
    1556,
    1605,
    1150,
    1143,
    1419,
    1426,
    1157,
    1563,
    1064,
    1570,
    1433,
    1440,
    1071,
    1577,
    1234,
    1584,
    1447,
    1454,
    1241,
]

light_id = [
    3699,
    3721,
    3710,
    3754,
    3743,
    3732,
    3787,
    3798,
    3765,
    3552,
    3611,
    3600,
    3688,
    3677,
    3666,
    3574,
    3633,
    3622,
    3655,
    3644,
    3589,
]


# Init. global variables about Pose
pos_lane_id = 0
pos_x = 0.0
pos_y = 0.0
pos_z = 0.0


session = zenoh.open()


class SignalPub(Node):
    def __init__(self, nodeName):
        super().__init__(nodeName)
        qos_profile = QoSProfile(
            reliability=2,  # RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
            history=1,  # RMW_QOS_POLICY_HISTORY_KEEP_LAST
            depth=1,
        )
        self.publication = self.create_publisher(TrafficSignalArray, 'perception/traffic_light_recognition/traffic_signals', 1)
        self.subscription = self.create_subscription(
            PathWithLaneId,
            'planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id',
            self.signal_callback,
            qos_profile=qos_profile,
        )
        self.pose_subscription = self.create_subscription(VehicleKinematics, 'api/vehicle/kinematics', self.pose_callback, 10)
        self.subscription
        self.pose_subscription

        self.pose_publisher = session.declare_publisher(f'vehicle/{args.vehicle}/pose')
        self.red_signal = self.publish_red_signal()

    def publish_red_signal(self):
        global light_id

        # Turn all traffic lights into red
        for id in light_id:
            color = (1, Shape.CIRCLE.value, Status.ON.value, confidence)
            traffic_signals = self.traffic_signals_gen(id, color)
            self.publication.publish(traffic_signals)
            selector, target, color = self.query_light_status(id)
            _replies = session.get(selector, zenoh.Queue(), target=target, value='red')

    def publish_pose(self):
        global pos_x, pos_y, pos_z, pos_lane_id

        pose = {'lane_id': pos_lane_id, 'position': {'x': pos_x, 'y': pos_y, 'z': pos_z}}

        self.pose_publisher.put(pose)

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
                    # print(closest_lane[0])
                    pos_lane_id = closest_lane[0]
                    idx = lane_id.index(d.lane_ids[0])
                    tl_id = light_id[idx]
                    selector, target, color = self.query_light_status(tl_id)

                    # Sync Autoware and Carla's traffic light signal
                    tl_color = (color_dict[color], Shape.CIRCLE.value, Status.ON.value, confidence)
                    traffic_signals = self.traffic_signals_gen(tl_id, tl_color)

                    self.publication.publish(traffic_signals)
                    self.publish_pose()
                    break
            else:
                pass

    def stamp_gen(self, frame_id=''):
        stamp = Time()
        t = self.get_clock().now()
        stamp = t.to_msg()
        return stamp

    def traffic_signal_element_gen(self, state):
        tse = TrafficLight()
        tse.color = state[0]
        tse.shape = state[1]
        tse.status = state[2]
        tse.confidence = state[3]
        # print(tse)
        return tse

    def traffic_signal_gen(self, tl_id, state):
        ts = TrafficSignal()
        ts.map_primitive_id = tl_id
        ts.lights.append(self.traffic_signal_element_gen(state))
        return ts

    def traffic_signals_gen(self, tl_id, state):
        stamp = self.stamp_gen()
        ts = self.traffic_signal_gen(tl_id, state)
        traffic_signals = TrafficSignalArray()
        traffic_signals.header.stamp = stamp
        # traffic_signals.header.seq = 0
        # traffic_signals.frame_id = ''
        traffic_signals.signals.append(ts)
        return traffic_signals

    def query_light_status(self, tl_id):
        # Match intersection ID and traffic light ID
        intersection_id = {
            3699: 'A',
            3721: 'A',
            3710: 'A',  # A
            3732: 'B',
            3754: 'B',
            3743: 'B',  # B
            3765: 'C',
            3787: 'C',
            3798: 'C',  # C
            3552: 'D',
            3611: 'D',
            3600: 'D',  # D
            3677: 'E',
            3666: 'E',
            3688: 'E',  # E
            3574: 'F',
            3633: 'F',
            3622: 'F',  # F
            3644: 'G',
            3589: 'G',
            3655: 'G',  # G
        }

        selector = f'intersection/{intersection_id[tl_id]}/traffic_light/' + str(tl_id) + '/state'

        target = {
            'ALL': QueryTarget.ALL(),
            'BEST_MATCHING': QueryTarget.BEST_MATCHING(),
            'ALL_COMPLETE': QueryTarget.ALL_COMPLETE(),
        }.get('BEST_MATCHING')

        replies = session.get(selector, zenoh.Queue(), target=target, value=None)

        if replies:
            for reply in replies.receiver:
                try:
                    payload = reply.ok.payload.decode('utf-8')
                    # print(">> Received ('{}': '{}')"
                    #     .format(reply.ok.key_expr, reply.ok.payload.decode("utf-8")))
                except Exception as _e:
                    payload = reply.err.payload.decode('utf-8')
                    # print(">> Received (ERROR: '{}')"
                    #     .format(reply.err.payload.decode("utf-8")))

        return selector, target, payload


def main(args=args):
    rclpy.init()
    node = SignalPub('v2x_light')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
