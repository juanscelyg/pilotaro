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
Flight check of one or several robots flown by pilotaro (simulation or real, use with care).

All the robots fly the same sequence at the same time, and every step has a pass criterion:
  1. takeoff before arming is rejected
  2. arming, takeoff: LOITER at home + takeoff_altitude
  3. hover: position held (max deviation) and rotor speeds without saturation
  4. offboard: a 3D goal (dx, dy, dz) is reached and held, with the overshoot measured
  5. yaw: the goal heading is reached
  6. landing: touchdown and DISARMED
Underwater vehicles (optional):
  0. --expect-float Z: before arming, the vehicle floats still at z = Z (the surface)
  7. --expect-ascent V: after landing on the bottom, a positively buoyant vehicle rises by
     itself at the terminal speed V given by its net buoyancy and its damping

Exit code 0 if every check of every robot passed.

  flight_check.py uav1 uav2 [--takeoff-altitude 2.0] [--goal 2 1 1] [--yaw 1.57]
  flight_check.py rov --takeoff-altitude -2 --expect-float -0.09 --expect-ascent 0.17
"""

import argparse
import math
import sys
import time

from actuator_msgs.msg import Actuators
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class Robot:

    def __init__(self, node, name):
        self.name = name
        self.odom = None
        self.mode = None
        self.motors = None
        self.goal = None  # (x, y, z, yaw) published on offboard/pose while set
        self.results = []
        ns = '/' + name
        node.create_subscription(
            Odometry, f'{ns}/odometry', self.odom_cb, qos_profile_sensor_data)
        node.create_subscription(
            String, f'{ns}/mode', self.mode_cb,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(Actuators, f'{ns}/command/motor_speed', self.motor_cb, 10)
        self.pose_pub = node.create_publisher(PoseStamped, f'{ns}/offboard/pose', 10)
        self.clients = {s: node.create_client(Trigger, f'{ns}/{s}')
                        for s in ('arming', 'takeoff', 'landing', 'loiter', 'offboard')}

    def odom_cb(self, msg):
        self.odom = msg

    def mode_cb(self, msg):
        self.mode = msg.data

    def motor_cb(self, msg):
        self.motors = list(msg.velocity)

    def position(self):
        p = self.odom.pose.pose.position
        return p.x, p.y, p.z

    def yaw(self):
        return yaw_of(self.odom.pose.pose.orientation)

    def speed(self):
        v = self.odom.twist.twist.linear
        return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

    def check(self, name, ok, detail):
        self.results.append((name, ok, detail))
        print(f'[{self.name}] {"PASS" if ok else "FAIL"} {name}: {detail}', flush=True)
        return ok


class FlightCheck(Node):

    def __init__(self, names):
        super().__init__('flight_check', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.robots = [Robot(self, n) for n in names]
        self.create_timer(0.05, self.publish_goals)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_goals(self):
        for r in self.robots:
            if r.goal is None:
                continue
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            x, y, z, yaw = r.goal
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
            msg.pose.orientation.z = math.sin(yaw / 2.0)
            msg.pose.orientation.w = math.cos(yaw / 2.0)
            r.pose_pub.publish(msg)

    def spin_for(self, seconds, every=None):
        """Spin for seconds of the node clock (simulation time), calling every() each step."""
        end = self.now() + seconds
        wall_end = time.monotonic() + 10.0 * seconds + 10.0
        while rclpy.ok() and self.now() < end and time.monotonic() < wall_end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if every:
                every()

    def wait(self, condition, timeout):
        end = self.now() + timeout
        wall_end = time.monotonic() + 10.0 * timeout + 10.0
        while rclpy.ok() and not condition():
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.now() > end or time.monotonic() > wall_end:
                return False
        return True

    def call(self, robot, service):
        client = robot.clients[service]
        if not client.wait_for_service(timeout_sec=10.0):
            return False, 'service not available'
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        res = future.result()
        return (res.success, res.message) if res else (False, 'no answer')


def run(node, args):
    robots = node.robots
    # With use_sim_time the clock is 0 until the first /clock message
    wall_end = time.monotonic() + 30.0
    while rclpy.ok() and node.now() == 0.0 and time.monotonic() < wall_end:
        rclpy.spin_once(node, timeout_sec=0.05)
    ready = node.wait(
        lambda: all(r.odom is not None and r.mode is not None for r in robots), 30.0)
    if not ready:
        for r in robots:
            r.check('ready', False, f'odometry {r.odom is not None}, mode {r.mode}')
        return
    node.spin_for(1.0)
    home = {r.name: r.position() for r in robots}

    # 0. floating at the surface
    if args.expect_float is not None:
        for r in robots:
            z = r.position()[2]
            r.check('floats at the surface',
                    abs(z - args.expect_float) < 0.05 and r.speed() < 0.05,
                    f'z {z:.3f} m (expected {args.expect_float:.3f}), speed {r.speed():.3f} m/s')

    # 1. takeoff before arming
    for r in robots:
        ok, msg = node.call(r, 'takeoff')
        r.check('takeoff rejected while DISARMED', not ok, msg)

    # 2. arming and takeoff
    for r in robots:
        ok, msg = node.call(r, 'arming')
        r.check('arming', ok, msg)
        ok, msg = node.call(r, 'takeoff')
        r.check('takeoff accepted', ok, msg)
    node.wait(lambda: all(r.mode == 'LOITER' for r in robots), 30.0)
    node.spin_for(3.0)
    for r in robots:
        target = home[r.name][2] + args.takeoff_altitude
        z = r.position()[2]
        r.check('takeoff altitude', r.mode == 'LOITER' and abs(z - target) < args.tolerance,
                f'mode {r.mode}, z {z:.3f} m, target {target:.3f} m')

    # 3. hover
    hold = {r.name: r.position() for r in robots}
    deviation = {r.name: 0.0 for r in robots}
    speeds = {r.name: [] for r in robots}

    def sample_hover():
        for r in robots:
            deviation[r.name] = max(deviation[r.name], math.dist(r.position(), hold[r.name]))
            if r.motors:
                speeds[r.name].append(r.motors)

    node.spin_for(args.hover_time, sample_hover)
    for r in robots:
        r.check('hover hold', deviation[r.name] < args.tolerance,
                f'max deviation {deviation[r.name]:.3f} m in {args.hover_time:.0f} s')
        if speeds[r.name]:
            n = len(speeds[r.name][0])
            mean = [sum(s[i] for s in speeds[r.name]) / len(speeds[r.name]) for i in range(n)]
            peak = max(max(abs(v) for v in s) for s in speeds[r.name])
            r.check('hover rotor speeds', peak < args.max_rotor_speed,
                    'mean [' + ', '.join(f'{m:.0f}' for m in mean) +
                    f'] rad/s, peak {peak:.0f} rad/s')

    # 4. offboard to a 3D goal
    dx, dy, dz = args.goal
    goals = {}
    for r in robots:
        x, y, z = r.position()
        goals[r.name] = (x + dx, y + dy, z + dz, r.yaw())
        r.goal = (x, y, z, r.yaw())  # stream the current pose first
    node.spin_for(0.5)
    for r in robots:
        ok, msg = node.call(r, 'offboard')
        r.check('offboard accepted', ok, msg)
        r.goal = goals[r.name]
    start = {r.name: r.position() for r in robots}
    overshoot = {r.name: 0.0 for r in robots}

    def sample_overshoot():
        for r in robots:
            g = goals[r.name][:3]
            s = start[r.name]
            p = r.position()
            axis = [g[i] - s[i] for i in range(3)]
            length = math.sqrt(sum(a * a for a in axis)) or 1.0
            progress = sum((p[i] - s[i]) * axis[i] for i in range(3)) / length
            overshoot[r.name] = max(overshoot[r.name], progress - length)

    node.spin_for(args.goal_time, sample_overshoot)
    for r in robots:
        error = math.dist(r.position(), goals[r.name][:3])
        r.check('offboard goal', r.mode == 'OFFBOARD' and error < args.tolerance,
                f'error {error:.3f} m, overshoot {overshoot[r.name]:.3f} m, '
                f'speed {r.speed():.2f} m/s')

    # 5. yaw
    for r in robots:
        x, y, z, yaw = goals[r.name]
        goals[r.name] = (x, y, z, yaw + args.yaw)
        r.goal = goals[r.name]
    node.spin_for(args.goal_time)
    for r in robots:
        error = abs(angle_diff(r.yaw(), goals[r.name][3]))
        r.check('yaw goal', error < 0.05,
                f'error {math.degrees(error):.1f} deg, position error '
                f'{math.dist(r.position(), goals[r.name][:3]):.3f} m')

    # 6. landing
    for r in robots:
        ok, msg = node.call(r, 'landing')
        r.check('landing accepted', ok, msg)
        r.goal = None
    node.wait(lambda: all(r.mode == 'DISARMED' for r in robots), args.landing_time)
    for r in robots:
        r.check('touchdown and DISARMED', r.mode == 'DISARMED',
                f'mode {r.mode}, z {r.position()[2]:.3f} m')

    # 7. passive ascent: buoyancy against damping
    if args.expect_ascent is not None:
        node.spin_for(8.0)  # reach the terminal speed
        start = {r.name: r.position()[2] for r in robots}
        node.spin_for(5.0)
        for r in robots:
            speed = (r.position()[2] - start[r.name]) / 5.0
            error = abs(speed - args.expect_ascent) / args.expect_ascent
            r.check('passive ascent', error < 0.15,
                    f'{speed:.3f} m/s (expected {args.expect_ascent:.3f}, '
                    f'error {100 * error:.0f} %)')


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('robots', nargs='+', help='robot names (namespaces)')
    parser.add_argument('--takeoff-altitude', type=float, default=2.0)
    parser.add_argument('--goal', type=float, nargs=3, default=[2.0, 1.0, 1.0],
                        metavar=('DX', 'DY', 'DZ'), help='offboard goal from the hover point')
    parser.add_argument('--yaw', type=float, default=math.pi / 2,
                        help='heading change in offboard [rad]')
    parser.add_argument('--tolerance', type=float, default=0.15, help='position tolerance [m]')
    parser.add_argument('--hover-time', type=float, default=5.0)
    parser.add_argument('--goal-time', type=float, default=8.0)
    parser.add_argument('--max-rotor-speed', type=float, default=float('inf'),
                        help='hover rotor speeds must stay below this [rad/s]')
    parser.add_argument('--landing-time', type=float, default=60.0,
                        help='max time to land and disarm [s]')
    parser.add_argument('--expect-float', type=float, metavar='Z',
                        help='underwater: z at which the disarmed vehicle floats')
    parser.add_argument('--expect-ascent', type=float, metavar='V',
                        help='underwater: terminal speed of the passive ascent [m/s]')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = FlightCheck(args.robots)
    try:
        run(node, args)
    finally:
        failed = [(r.name, n) for r in node.robots for n, ok, _ in r.results if not ok]
        total = sum(len(r.results) for r in node.robots)
        print(f'\n{total - len(failed)}/{total} checks passed', flush=True)
        for name, check in failed:
            print(f'  FAILED [{name}] {check}', flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
    return 1 if failed or total == 0 else 0


if __name__ == '__main__':
    sys.exit(main())
