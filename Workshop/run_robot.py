#!/usr/bin/env python3
"""Start the robot program, like Robot() in a Jupyter notebook.

The frontend starts this script with its "Switch Robot On" button (see robot_server.py).
The robot then waits for the "Start Driving" button before it drives.

  python3 run_robot.py [--intersection] [--duck-evasion] [--settings robot_settings.json]
"""
import argparse
import json
import os
import signal
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rospy
from std_msgs.msg import String

from Robot.robot import Robot


def apply_settings(robot, settings, duck_evasion):
    """Give the robot the parameters that were last set in the frontend."""
    if 'yellow' in settings:
        robot.lane_detector.yellow_line_callback(String(data=json.dumps(settings['yellow'])))
    if 'white' in settings:
        robot.lane_detector.white_line_callback(String(data=json.dumps(settings['white'])))
    if 'pid' in settings:
        robot.lane_controller.pid_callback(String(data=json.dumps(settings['pid'])))
    if 'duck' in settings:
        robot.duckie_detector.settings_callback(String(data=json.dumps(settings['duck'])))
    # the start option wins over the saved checkbox of the duck page
    robot.duckie_detector.evade = duck_evasion


def load_settings(path):
    try:
        with open(path) as file:
            return json.load(file)
    except (OSError, ValueError):
        return {}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--intersection', action='store_true', help='drive through intersections')
    parser.add_argument('--duck-evasion', action='store_true', help='drive around ducks instead of stopping')
    parser.add_argument('--settings', default='/workspace/settings/robot_settings.json')
    args = parser.parse_args()

    robot = Robot()
    robot.use_intersection = args.intersection

    settings = load_settings(args.settings)
    apply_settings(robot, settings, args.duck_evasion)
    print(f'intersections: {"on" if args.intersection else "off"}, '
          f'duck evasion: {"on" if args.duck_evasion else "off (ducks are ignored)"}')
    print(f'parameters loaded: {", ".join(settings) if settings else "none saved yet, using defaults"}')

    def shutdown(signum, frame):
        print('stopping robot')
        robot.stop()
        rospy.signal_shutdown('stopped from the frontend')

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print('robot is on, waiting for the Start Driving button')
    rospy.spin()


if __name__ == '__main__':
    main()
