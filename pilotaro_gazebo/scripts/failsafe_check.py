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
Failsafe check: injects failures into one robot flown by pilotaro (simulation only) and
checks the reaction. The failsafe parameters are set at run time on states_node.

  1. pre-arm: with the battery at 20 % arming is rejected (the reason names the battery);
     at 90 % it arms and takes off
  2. geofence (radius 3 m): an offboard goal 6 m away stops at the fence
  3. ground station link (heartbeat_timeout 1 s): when the heartbeats stop the vehicle
     holds its position (LOITER), offboard is rejected and the failsafe is reported on
     the failsafe topic; it clears when they come back
  4. battery critical (5 %): the vehicle lands and disarms; takeoff and offboard are
     rejected meanwhile

Exit code 0 if every check passed.

  failsafe_check.py uav1
"""

import argparse
import math
import os
import sys

import rclpy
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Empty, String

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from flight_check import FlightCheck  # noqa: E402


class FailsafeCheck(FlightCheck):

    def __init__(self, name):
        super().__init__([name])
        self.robot = self.robots[0]
        ns = '/' + name
        self.battery = None          # charge 0-1 published at 10 Hz while set
        self.heartbeat = False       # heartbeats published at 10 Hz while True
        self.failsafe = None
        self.battery_pub = self.create_publisher(BatteryState, f'{ns}/battery', 10)
        self.heartbeat_pub = self.create_publisher(Empty, f'{ns}/heartbeat', 10)
        self.create_subscription(
            String, f'{ns}/failsafe', self.failsafe_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.params = self.create_client(SetParameters, f'{ns}/states_node/set_parameters')
        self.create_timer(0.1, self.publish_inputs)

    def failsafe_cb(self, msg):
        self.failsafe = msg.data

    def publish_inputs(self):
        if self.battery is not None:
            msg = BatteryState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.percentage = float(self.battery)
            self.battery_pub.publish(msg)
        if self.heartbeat:
            self.heartbeat_pub.publish(Empty())

    def set_failsafe(self, **values):
        """Set failsafe.<name> parameters of states_node (double values)."""
        if not self.params.wait_for_service(timeout_sec=10.0):
            return False
        req = SetParameters.Request()
        for name, value in values.items():
            req.parameters.append(ParameterMsg(
                name=f'failsafe.{name}',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                     double_value=float(value))))
        future = self.params.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        res = future.result()
        return res is not None and all(r.successful for r in res.results)

    def horizontal(self):
        x, y, _ = self.robot.position()
        return math.hypot(x, y)


def run(node, args):
    r = node.robot
    if not node.wait(lambda: node.now() > 0.0 and r.odom is not None and r.mode is not None,
                     30.0):
        r.check('ready', False, 'no odometry or mode')
        return
    node.wait_still([r], 20.0)
    x0, y0, z0 = r.position()

    # 1. Pre-arm battery check
    node.battery = 0.2
    node.spin_for(0.5)
    ok, msg = node.call(r, 'arming')
    r.check('arming rejected with low battery', not ok and 'battery' in msg, msg)
    node.battery = 0.9
    node.spin_for(0.5)
    ok, msg = node.call(r, 'arming')
    r.check('arming with charged battery', ok, msg)
    ok, msg = node.call(r, 'takeoff')
    reached = ok and node.wait(lambda: r.mode == 'LOITER', 25.0)
    r.check('takeoff', reached, f'mode {r.mode}')
    if not reached:
        return
    z = r.position()[2]

    # 2. Geofence: an offboard goal outside stops at the fence
    r.check('geofence set', node.set_failsafe(geofence_radius=3.0), 'radius 3 m')
    r.goal = (x0 + 6.0, y0, z, 0.0)
    node.spin_for(0.5)
    ok, msg = node.call(r, 'offboard')
    max_d = [0.0]
    node.spin_for(10.0, every=lambda: max_d.__setitem__(0, max(max_d[0], node.horizontal())))
    d = node.horizontal()
    r.check('stops at the geofence', ok and 2.7 < d < 3.3 and max_d[0] < 3.4,
            f'{msg}; distance {d:.2f} m (max {max_d[0]:.2f} m, fence 3 m)')
    r.goal = (x0, y0, z, 0.0)
    node.spin_for(6.0)

    # 3. Ground station link
    node.heartbeat = True
    node.spin_for(0.5)
    r.check('heartbeat timeout set', node.set_failsafe(heartbeat_timeout=1.0), '1 s')
    node.spin_for(2.0)
    in_offboard = r.mode == 'OFFBOARD'
    node.heartbeat = False
    t_stop = node.now()
    held = node.wait(lambda: r.mode == 'LOITER', 5.0)
    delay = node.now() - t_stop
    r.check('link loss holds position', in_offboard and held and 0.9 < delay < 2.0,
            f'mode {r.mode} {delay:.1f} s after the last heartbeat')
    reported = node.wait(lambda: node.failsafe == 'heartbeat', 1.0)
    r.check('failsafe reported', reported, f'failsafe "{node.failsafe}"')
    ok, msg = node.call(r, 'offboard')
    r.check('offboard rejected without link', not ok and 'heartbeat' in msg, msg)
    node.heartbeat = True
    cleared = node.wait(lambda: node.failsafe == '', 3.0)
    r.check('link back clears it', cleared, f'failsafe "{node.failsafe}"')
    node.set_failsafe(heartbeat_timeout=0.0)
    r.goal = None

    # 4. Battery critical
    node.battery = 0.05
    landing = node.wait(lambda: r.mode == 'LANDING', 3.0)
    node.wait(lambda: node.failsafe == 'battery_critical', 1.0)
    r.check('critical battery lands', landing, f'mode {r.mode}, failsafe "{node.failsafe}"')
    ok, msg = node.call(r, 'takeoff')
    ok2, msg2 = node.call(r, 'offboard')
    r.check('takeoff and offboard rejected', not ok and not ok2, f'{msg} / {msg2}')
    disarmed = node.wait(lambda: r.mode == 'DISARMED', 30.0)
    r.check('touchdown and DISARMED', disarmed, f'mode {r.mode}, z {r.position()[2]:.2f} m')
    node.set_failsafe(geofence_radius=0.0)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('robot', help='robot name (namespace)')
    args = parser.parse_args(rclpy.utilities.remove_ros_args()[1:])
    rclpy.init()
    node = FailsafeCheck(args.robot)
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
