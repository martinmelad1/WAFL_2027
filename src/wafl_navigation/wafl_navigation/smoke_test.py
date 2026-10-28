#!/usr/bin/env python3
"""smoke_test - drives every actuator through base_controller (JOINT mode) and checks /joint_states.

Needs: Gazebo sim + base_controller (sim:=true) running. The robot MOVES a little (< ~1 m): clear the area.
Checks per joint: reaches the target, does not jitter (steering), rolls at the commanded speed (wheels).
Exit code 0 = no FAIL, 1 = at least one FAIL.
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int32, String

LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def tail(samples, secs=1.0):
    return [s for s in samples if s[0] >= samples[-1][0] - secs] if samples else []


def p2p(samples):
    vals = [p for _, p in samples]
    return max(vals) - min(vals)


def rates(samples):
    return [(b[1] - a[1]) / (b[0] - a[0]) for a, b in zip(samples, samples[1:]) if b[0] > a[0]]


class SmokeTest(Node):
    def __init__(self, a):
        super().__init__('wafl_smoke_test')
        self.a = a
        self.pos, self.hist, self.fork, self.results = {}, {}, [], []
        self.cmd = [0.0, 0.0, 0.0, 0.0]   # [alpha_deg, beta_deg, v_left, v_right]
        self.lift = None
        self.create_subscription(JointState, '/joint_states', self._js, qos_profile_sensor_data)
        self.create_subscription(Float32, '/fork_height', self._fh, qos_profile_sensor_data)
        self.pub_mode = self.create_publisher(String, '/gui/mode', LATCHED)
        self.pub_estop = self.create_publisher(Bool, '/e_stop', LATCHED)
        self.pub_drive = self.create_publisher(String, '/drive_mode', LATCHED)
        self.pub_joint = self.create_publisher(Float32MultiArray, '/manual_joint_cmd', 10)
        self.pub_lift = self.create_publisher(Int32, '/lift_cmd', 10)

    # ---------------------------------------------------------------- io
    def _js(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 or time.monotonic()
        for name, p in zip(msg.name, msg.position):
            self.pos[name] = p
            h = self.hist.setdefault(name, [])
            if not h or t - h[-1][0] >= 0.05 or t < h[-1][0]:     # decimate to ~20 Hz of sim time
                h.append((t, p))

    def _fh(self, msg):
        self.fork.append((time.monotonic(), msg.data))

    def _send(self):
        self.pub_joint.publish(Float32MultiArray(data=[float(x) for x in self.cmd]))
        if self.lift is not None:
            self.pub_lift.publish(Int32(data=int(self.lift)))

    def phase(self, secs, cmd=None):
        """Publish the command at 10 Hz for `secs`; return {joint: samples recorded during the phase}."""
        if cmd is not None:
            self.cmd = list(cmd)
        marks = {n: len(h) for n, h in self.hist.items()}
        end, next_pub = time.monotonic() + secs, 0.0
        while time.monotonic() < end and rclpy.ok():
            if time.monotonic() >= next_pub:
                self._send()
                next_pub = time.monotonic() + 0.1
            rclpy.spin_once(self, timeout_sec=0.02)
        return {n: self.hist[n][marks.get(n, 0):] for n in self.hist}

    def record(self, test, joint, status, detail):
        self.results.append(status)
        print(f'[{status:4}] {test:30} {joint:14} {detail}', flush=True)

    # ---------------------------------------------------------------- checks
    def check_angle(self, test, joints, target_deg, data):
        for j in joints:
            s = tail(data.get(j, []))
            if len(s) < 5:
                self.record(test, j, 'FAIL', 'no /joint_states data')
                continue
            err = math.degrees(sum(p for _, p in s) / len(s)) - target_deg
            jit = math.degrees(p2p(s))
            ok = abs(err) <= self.a.angle_tol and jit <= self.a.jitter_tol
            self.record(test, j, 'PASS' if ok else 'FAIL', f'error {err:+.2f} deg, jitter {jit:.2f} deg p-p')

    def check_wheel(self, test, joint, v_cmd, data):
        s = tail(data.get(joint, []))
        if len(s) < 5:
            self.record(test, joint, 'FAIL', 'no /joint_states data')
            return
        expect = v_cmd / self.a.wheel_radius
        rate = (s[-1][1] - s[0][1]) / (s[-1][0] - s[0][0])
        rr = rates(s)
        ripple = (max(rr) - min(rr)) if rr else 0.0
        ok = abs(rate) < 0.3 if abs(expect) < 1e-6 else (rate * expect > 0 and abs(rate - expect) <= 0.35 * abs(expect))
        status = 'PASS' if ok else 'FAIL'
        if ok and abs(expect) > 1e-6 and ripple > 0.5 * abs(expect):
            status = 'WARN'
        self.record(test, joint, status, f'{rate:+.2f} rad/s (expected {expect:+.2f}), ripple {ripple:.2f} rad/s')

    # ---------------------------------------------------------------- the test
    def run(self):
        a = self.a
        front, rear = a.front_joints.split(','), a.rear_joint
        wl, wr = a.wheel_joints.split(',')

        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < 15.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.pos and self.pub_joint.get_subscription_count() > 0:
                break
        else:
            print('ABORT: no /joint_states or base_controller not running (nobody subscribes /manual_joint_cmd)')
            return 1
        missing = [j for j in front + [rear, wl, wr] if j not in self.pos]
        if missing:
            print(f'ABORT: joints {missing} not in /joint_states. Available: {sorted(self.pos)}')
            return 1

        self.pub_estop.publish(Bool(data=False))
        self.pub_mode.publish(String(data='MANUAL'))
        self.pub_drive.publish(String(data='JOINT'))
        print(f'Waiting {a.settle:.1f} s for base_controller to recentre and enter JOINT mode...')
        self.phase(a.settle, [0, 0, 0, 0])

        for tgt in (30.0, -30.0, 0.0):                       # front steering
            self.check_angle(f'front steer -> {tgt:+.0f} deg', front, tgt, self.phase(2.5, [tgt, 0, 0, 0]))
        for tgt in (30.0, -30.0, 0.0):                       # rear steering
            self.check_angle(f'rear steer -> {tgt:+.0f} deg', [rear], tgt, self.phase(2.5, [0, tgt, 0, 0]))

        if not a.skip_wheels:
            L, W = a.wheelbase, a.track
            for name, v, w in (('straight forward', 0.2, 0.0), ('straight reverse', -0.2, 0.0),
                               ('left arc', 0.2, 0.4), ('right arc', 0.2, -0.4)):
                vl, vr = v - w * W / 2.0, v + w * W / 2.0
                beta = math.degrees(math.atan2(-w * L * (1.0 if v >= 0 else -1.0), abs(v)))
                self.phase(1.5, [0, beta, 0, 0])             # pre-position the rear wheel
                data = self.phase(2.5, [0, beta, vl, vr])
                self.check_wheel(f'{name}: left wheel', wl, vl, data)
                self.check_wheel(f'{name}: right wheel', wr, vr, data)
                self.phase(1.5, [0, 0, 0, 0])

        if not a.skip_lift:                                  # lift: 1 = UP, 2 = DOWN
            for name, cmd, sign in (('lift UP', 1, +1), ('lift DOWN', 2, -1)):
                n0 = len(self.fork)
                self.lift = cmd
                self.phase(2.0, [0, 0, 0, 0])
                self.lift = 0
                self._send()
                self.lift = None
                self.phase(0.5)
                got = self.fork[n0:]
                if len(got) < 2:
                    self.record(name, '/fork_height', 'SKIP',
                                'no /fork_height (2025 URDF has no fork joint, sim_fork_adapter missing)')
                else:
                    delta = got[-1][1] - got[0][1]
                    ok = delta * sign > a.lift_min_mm
                    self.record(name, '/fork_height', 'PASS' if ok else 'FAIL', f'{delta:+.1f} mm')

        self.phase(0.5, [0, 0, 0, 0])
        self.pub_drive.publish(String(data='NORMAL'))
        n = {s: self.results.count(s) for s in ('PASS', 'WARN', 'FAIL', 'SKIP')}
        print(f'\nSUMMARY: {n["PASS"]} PASS, {n["WARN"]} WARN, {n["FAIL"]} FAIL, {n["SKIP"]} SKIP')
        return 1 if n['FAIL'] else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--front-joints', default='front_left_steer_joint,front_right_steer_joint')
    ap.add_argument('--rear-joint', default='caster_joint')
    ap.add_argument('--wheel-joints', default='front_left_wheel_joint,front_right_wheel_joint', help='left,right')
    ap.add_argument('--wheel-radius', type=float, default=0.1)
    ap.add_argument('--track', type=float, default=0.3395)
    ap.add_argument('--wheelbase', type=float, default=0.462)
    ap.add_argument('--settle', type=float, default=3.0, help='s to wait for recentre + mode change')
    ap.add_argument('--angle-tol', type=float, default=2.0, help='deg')
    ap.add_argument('--jitter-tol', type=float, default=0.5, help='deg peak-to-peak while holding')
    ap.add_argument('--lift-min-mm', type=float, default=5.0)
    ap.add_argument('--skip-wheels', action='store_true')
    ap.add_argument('--skip-lift', action='store_true')
    args = ap.parse_args(remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node, code = SmokeTest(args), 1
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 130
    finally:
        try:
            node.cmd, node.lift = [0.0] * 4, 0
            node._send()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()