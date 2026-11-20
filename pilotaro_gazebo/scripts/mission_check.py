#!/usr/bin/env python3
# Copyright 2026 Juan S. Cely G.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Mission check: one multirotor flown by pilotaro (simulation or real, use with care) flies a
waypoint mission from the ground and returns to the arming point.

  1. the mission (nav_msgs/Path on mission) is accepted and started while ARMED: it takes
     off first
  2. every waypoint is reached in order (mission/current), within --tolerance
  3. the legs are straight (cross-track error) and flown at mission.speed at most
  4. after the last waypoint (mission.end_action = rtl) it climbs to mission.rtl_altitude
     over home, returns and lands at the arming point, then disarms

The positions are measured with the odometry topic (the Gazebo ground truth in simulation).
Exit code 0 if every check passed.

  mission_check.py uav1 [--tolerance 0.35]
"""

import argparse
import math
import os
import sys

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from flight_check import FlightCheck  # noqa: E402

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def to_segment(p, a, b):
    ab = [b[i] - a[i] for i in range(3)]
    ap = [p[i] - a[i] for i in range(3)]
    n2 = sum(v * v for v in ab)
    t = 0.0 if n2 == 0 else max(0.0, min(1.0, sum(ap[i] * ab[i] for i in range(3)) / n2))
    return math.dist(p, [a[i] + t * ab[i] for i in range(3)])


class MissionCheck(FlightCheck):

    def __init__(self, name):
        super().__init__([name])
        self.robot = self.robots[0]
        ns = '/' + name
        self.current = None
        self.mission_pub = self.create_publisher(Path, f'{ns}/mission', LATCHED)
        self.create_subscription(Int32, f'{ns}/mission/current', self.current_cb, LATCHED)
        self.params = self.create_client(SetParameters, f'{ns}/states_node/set_parameters')
        for service in ('mission', 'rtl'):
            self.robot.clients[service] = self.create_client(Trigger, f'{ns}/{service}')

    def current_cb(self, msg):
        self.current = msg.data

    def set_param(self, name, value):
        if not self.params.wait_for_service(timeout_sec=10.0):
            return False
        v = (ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)
             if isinstance(value, str) else
             ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)))
        future = self.params.call_async(
            SetParameters.Request(parameters=[ParameterMsg(name=name, value=v)]))
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        res = future.result()
        return res is not None and all(r.successful for r in res.results)

    def publish_mission(self, waypoints):
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        for x, y, z, yaw in waypoints:
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x, p.pose.position.y, p.pose.position.z = x, y, z
            if yaw is not None:
                p.pose.orientation.z = math.sin(yaw / 2.0)
                p.pose.orientation.w = math.cos(yaw / 2.0)
            msg.poses.append(p)
        self.mission_pub.publish(msg)


def run(node, args):
    r = node.robot
    if not node.wait(lambda: node.now() > 0.0 and r.odom is not None and r.mode is not None,
                     30.0):
        r.check('ready', False, 'no odometry or mode')
        return
    node.wait_still([r], 20.0)
    home = r.position()
    x0, y0, z0 = home
    wps = [(x0 + 4.0, y0, z0 + 3.0, None), (x0 + 4.0, y0 + 4.0, z0 + 3.0, math.pi / 2),
           (x0, y0 + 4.0, z0 + 2.0, None)]
    speed = 2.0
    ok = (node.set_param('mission.end_action', 'rtl') and
          node.set_param('mission.speed', speed) and node.set_param('mission.rtl_altitude', 5.0))
    r.check('mission parameters', ok, 'end_action rtl, speed 2 m/s, rtl_altitude 5 m')
    node.publish_mission(wps)
    node.spin_for(1.0)

    ok, msg = node.call(r, 'arming')
    r.check('arming', ok, msg)
    ok, msg = node.call(r, 'mission')
    r.check('mission accepted while ARMED', ok and r.mode in ('TAKEOFF', 'ARMED'), msg)
    started = node.wait(lambda: r.mode == 'MISSION', 30.0)
    r.check('takes off and starts', started, f'mode {r.mode}')
    if not started:
        return

    # Fly the mission: closest approach to each waypoint, cross-track error, speed
    closest = [math.inf] * len(wps)
    visited = []
    cross = [0.0]
    top_speed = [0.0]
    leg_from = [r.position()]

    def track():
        p = r.position()
        i = node.current
        if i is not None and i >= 0:
            if not visited or visited[-1] != i:
                if visited:
                    leg_from[0] = wps[visited[-1]][:3]
                visited.append(i)
            closest[i] = min(closest[i], math.dist(p, wps[i][:3]))
            # skip the first and last 0.6 m of each leg (corners)
            a, b = leg_from[0], wps[i][:3]
            if math.dist(p, a) > 0.6 and math.dist(p, b) > 0.6:
                cross[0] = max(cross[0], to_segment(p, a, b))
        v = r.odom.twist.twist.linear
        top_speed[0] = max(top_speed[0], math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z))
    node.wait(lambda: (track(), r.mode != 'MISSION')[1], 90.0)

    r.check('waypoints in order', visited == list(range(len(wps))), f'visited {visited}')
    for i, d in enumerate(closest):
        r.check(f'waypoint {i} reached', d < args.tolerance, f'closest {d:.2f} m')
    r.check('straight legs', cross[0] < 0.3, f'max cross-track error {cross[0]:.2f} m')
    r.check('mission speed', top_speed[0] < speed * 1.15,
            f'max speed {top_speed[0]:.2f} m/s (mission.speed {speed})')

    # Return to launch
    r.check('mission end: RTL', r.mode == 'RTL', f'mode {r.mode}')
    max_z = [0.0]
    node.wait(lambda: (max_z.__setitem__(0, max(max_z[0], r.position()[2])),
                       r.mode != 'RTL')[1], 60.0)
    r.check('climbs to the return height', max_z[0] > z0 + 4.7,
            f'max height {max_z[0] - z0:.2f} m over home (rtl_altitude 5 m)')
    r.check('lands at home', r.mode == 'LANDING', f'mode {r.mode}')
    disarmed = node.wait(lambda: r.mode == 'DISARMED', 30.0)
    x, y, _ = r.position()
    d = math.hypot(x - x0, y - y0)
    r.check('touchdown at home and DISARMED', disarmed and d < 0.5,
            f'mode {r.mode}, {d:.2f} m from the arming point')


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('robot', help='robot name (namespace)')
    parser.add_argument('--tolerance', type=float, default=0.35,
                        help='closest approach to each waypoint [m]')
    args = parser.parse_args(rclpy.utilities.remove_ros_args()[1:])
    rclpy.init()
    node = MissionCheck(args.robot)
    try:
        run(node, args)
    finally:
        results = node.robot.results
        passed = sum(ok for _, ok, _ in results)
        print(f'\n{passed}/{len(results)} checks passed')
        for name, ok, detail in results:
            if not ok:
                print(f'  FAILED {name}: {detail}')
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if results and passed == len(results) else 1)


if __name__ == '__main__':
    main()
