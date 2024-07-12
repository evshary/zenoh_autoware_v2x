#
# Copyright (c) 2022 ZettaScale Technology
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
# which is available at https://www.apache.org/licenses/LICENSE-2.0.
#
# SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
#
# Contributors:
#   ZettaScale Zenoh Team, <zenoh@zettascale.tech>
#

import argparse
import json

import zenoh
from zenoh import QueryTarget

# --- Command line argument parsing --- --- --- --- --- ---
parser = argparse.ArgumentParser(prog='controller.py', description='Carla traffic light controller.')
parser.add_argument('--mode', '-m', dest='mode', choices=['peer', 'client'], type=str, help='The zenoh session mode.')
parser.add_argument('--connect', '-e', dest='connect', metavar='ENDPOINT', action='append', type=str, help='Endpoints to connect to.')
parser.add_argument('--listen', '-l', dest='listen', metavar='ENDPOINT', action='append', type=str, help='Endpoints to listen on.')
parser.add_argument(
    '--target',
    '-t',
    dest='target',
    choices=['ALL', 'BEST_MATCHING', 'ALL_COMPLETE', 'NONE'],
    default='BEST_MATCHING',
    type=str,
    help='The target queryables of the query.',
)
parser.add_argument('--value', '-v', dest='value', type=str, help='An optional value to send in the query.')
parser.add_argument('--config', '-c', dest='config', metavar='FILE', type=str, help='A configuration file.')
parser.add_argument('--command', '--cmd', dest='command', choices=['get', 'set'], required=True, type=str, help='The query command.')
parser.add_argument('--light_id', '-i', dest='light_id', required=True, type=int, help="The traffic light's id")
parser.add_argument('--state', '-s', dest='state', choices=['green', 'red'], type=str, help="The light's state")

args = parser.parse_args()
conf = zenoh.Config.from_file(args.config) if args.config is not None else zenoh.Config()
if args.mode is not None:
    conf.insert_json5(zenoh.config.MODE_KEY, json.dumps(args.mode))
if args.connect is not None:
    conf.insert_json5(zenoh.config.CONNECT_KEY, json.dumps(args.connect))
if args.listen is not None:
    conf.insert_json5(zenoh.config.LISTEN_KEY, json.dumps(args.listen))

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

selector = f'intersection/{intersection_id[args.light_id]}/traffic_light/' + str(args.light_id) + '/state'

target = {
    'ALL': QueryTarget.ALL(),
    'BEST_MATCHING': QueryTarget.BEST_MATCHING(),
    'ALL_COMPLETE': QueryTarget.ALL_COMPLETE(),
}.get(args.target)

# Zenoh code  --- --- --- --- --- --- --- --- --- --- ---

# initiate logging
zenoh.init_logger()

print('Opening session...')
session = zenoh.open(conf)

print("Sending Query '{}'...".format(selector))

if args.command == 'get':
    replies = session.get(selector, zenoh.Queue(), target=target, value=None)
elif args.command == 'set' and args.state is not None:
    replies = session.get(selector, zenoh.Queue(), target=target, value=args.state)
else:
    print('Non-valid query.')
    replies = None

if replies:
    for reply in replies.receiver:
        try:
            print(">> Received ('{}': '{}')".format(reply.ok.key_expr, reply.ok.payload.decode('utf-8')))
        except Exception as _e:
            print(">> Received (ERROR: '{}')".format(reply.err.payload.decode('utf-8')))


session.close()
