#!/usr/bin/env python3
"""base_controller - motion arbiter + steering/drive kinematics for the WAFL forklift (README 4.3).

Geometry: 2 driven front wheels (steered together by alpha), 1 undriven steered rear wheel (beta).
Command point = midpoint of the front axle = base_footprint origin.
Angles follow REP-103 (positive = wheel points left); front_direction / rear_direction flip them on OUTPUT only.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Float32MultiArray, Float64, String

# /e_stop, /gui/mode, /drive_mode are latched (README section 7)
LATCHED = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
# best-effort subscriber: connects to reliable AND best-effort publishers
CMD_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
MODES = ('NORMAL', 'CRAB', 'JOINT')
EPS = 1e-3


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class BaseController(Node):
    def __init__(self):
        super().__init__('base_controller')
        p = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.sim = p('sim', True)
        self.W = p('track_width', 0.3395)
        self.L = p('wheelbase_L', 0.462)
        self.r = p('wheel_radius', 0.1)
        self.max_front = math.radians(p('max_front_angle_deg', 90.0))
        self.max_rear = math.radians(p('max_rear_angle_deg', 80.0))
        self.vmax = p('max_wheel_speed', 1.0)
        self.front_dir = p('front_direction', 1.0)
        self.rear_dir = p('rear_direction', 1.0)
        self.settle = p('steer_settle_time', 1.0)
        self.deadband = math.radians(p('steer_deadband_deg', 1.0))
        self.cmd_timeout = p('cmd_timeout', 0.5)
        self.auto_timeout = p('autonomy_timeout', 0.2)
        self.slew = math.radians(p('max_steer_rate_dps', 90.0))
        self.accel = p('max_wheel_accel', 2.0)
        rate = p('rate_hz', 50.0)

        # ---- state ----
        self.estop = False
        self.control_mode = 'MANUAL'      # until the first /gui/mode sample the robot never moves on its own
        self.requested = 'NORMAL'         # drive mode asked for by the operator
        self.active = 'NORMAL'            # drive mode actually in use
        self.pending_recentre = False
        self.stage = None                 # None | 'slew' | 'wait'  (recentring between modes)
        self.stage_end = 0.0
        self.reached_since = None         # CRAB: time the steering has been on target
        self.alpha = self.beta = 0.0      # rad, OUTPUT after slew limiting
        self.t_alpha = self.t_beta = 0.0  # rad, current targets (held when a source goes silent)
        self.vl = self.vr = 0.0           # m/s, OUTPUT after ramp
        self.twists = {}                  # source -> (Twist, receive time)
        self.joint_cmd = None             # ([alpha_deg, beta_deg, vl, vr], receive time)
        self.last_t = self.now_s()
        self.last_source = None

        # ---- inputs ----
        for key, topic in (('manual', '/manual_cmd_vel'), ('fork', '/fork_insertion/cmd_vel'),
                           ('dock', '/charging_dock/cmd_vel'), ('nav', '/cmd_vel')):
            self.create_subscription(Twist, topic, lambda m, k=key: self._twist_cb(k, m), CMD_QOS)
        self.create_subscription(Float32MultiArray, '/manual_joint_cmd', self._joint_cb, CMD_QOS)
        self.create_subscription(String, '/gui/mode', self._mode_cb, LATCHED)
        self.create_subscription(String, '/drive_mode', self._drive_cb, LATCHED)
        self.create_subscription(Bool, '/e_stop', self._estop_cb, LATCHED)

        # ---- outputs: sim OR hardware, never both ----
        if self.sim:
            self.pub_front = self.create_publisher(Float64, '/front_steer_cmd_pos', 10)
            self.pub_rear = self.create_publisher(Float64, '/castor_cmd_pos', 10)
            self.pub_fl = self.create_publisher(Float64, '/front_left_wheel_cmd_vel', 10)
            self.pub_fr = self.create_publisher(Float64, '/front_right_wheel_cmd_vel', 10)
        else:
            self.pub_front = self.create_publisher(Float32, '/target_front_angle', 10)
            self.pub_rear = self.create_publisher(Float32, '/target_angle', 10)
            self.pub_fl = self.create_publisher(Float32, '/target_speed_left', 10)
            self.pub_fr = self.create_publisher(Float32, '/target_speed_right', 10)

        self.create_timer(1.0 / rate, self._cycle)
        self.get_logger().info(
            f'base_controller up: sim={self.sim} W={self.W} L={self.L} r={self.r} - mode MANUAL/NORMAL')

    # ------------------------------------------------------------------ helpers
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _fresh(self, key, now, timeout):
        item = self.twists.get(key)
        if item is not None and 0.0 <= now - item[1] <= timeout:
            return item[0]
        return None

    # ------------------------------------------------------------------ callbacks
    def _twist_cb(self, key, msg):
        vals = (msg.linear.x, msg.linear.y, msg.angular.z)
        if all(math.isfinite(x) for x in vals):
            self.twists[key] = (msg, self.now_s())

    def _joint_cb(self, msg):
        if len(msg.data) >= 4 and all(math.isfinite(x) for x in msg.data[:4]):
            self.joint_cmd = (list(msg.data[:4]), self.now_s())

    def _mode_cb(self, msg):
        m = msg.data.strip().upper()
        if m not in ('MANUAL', 'AUTO'):
            self.get_logger().warn(f'/gui/mode: unknown value "{msg.data}"')
            return
        if m != self.control_mode:
            self.get_logger().info(f'Control mode: {self.control_mode} -> {m}')
        self.control_mode = m

    def _drive_cb(self, msg):
        m = msg.data.strip().upper()
        if m not in MODES:
            self.get_logger().warn(f'/drive_mode: unknown value "{msg.data}"')
            return
        if m != 'NORMAL' and (self.control_mode != 'MANUAL' or self.estop):
            self.get_logger().warn(f'/drive_mode {m} ignored: only allowed in MANUAL with e-stop clear')
            return
        self.requested = m

    def _estop_cb(self, msg):
        if msg.data != self.estop:
            self.get_logger().warn('E-STOP LATCHED' if msg.data else 'E-stop released')
        self.estop = msg.data

    # ------------------------------------------------------------------ arbitration
    def _select_twist(self, now):
        """Returns (source name, Twist or None)."""
        if self.control_mode == 'MANUAL':
            return 'manual', self._fresh('manual', now, self.cmd_timeout)
        recent = [(k, self.twists[k][1]) for k in ('fork', 'dock')
                  if self._fresh(k, now, self.auto_timeout) is not None]
        if recent:
            key = max(recent, key=lambda kv: kv[1])[0]
            return key, self.twists[key][0]
        return 'nav', self._fresh('nav', now, self.cmd_timeout)

    # ------------------------------------------------------------------ kinematics
    # every mode returns (v_left, v_right, alpha_target | None, beta_target | None, hard_stop)
    # None = keep the current target (hold). hard_stop = bypass the acceleration ramp.
    def _normal(self, tw):
        if tw is None:
            return 0.0, 0.0, 0.0, None, True
        v, w = tw.linear.x, tw.angular.z
        vl = v - w * self.W / 2.0
        vr = v + w * self.W / 2.0
        m = max(abs(vl), abs(vr))
        if m > self.vmax:                      # scale (v, w) together: the turn radius is preserved
            k = self.vmax / m
            v, w, vl, vr = v * k, w * k, vl * k, vr * k
        sgn = 1.0 if v >= 0.0 else -1.0
        beta = clamp(math.atan2(-w * self.L * sgn, abs(v)), -self.max_rear, self.max_rear)
        return vl, vr, 0.0, beta, False

    def _crab(self, tw):
        if tw is None:
            return 0.0, 0.0, None, None, True
        vx = 0.0 if abs(tw.linear.x) < EPS else tw.linear.x
        vy = tw.linear.y
        s = math.hypot(vx, vy)
        if s < EPS:
            return 0.0, 0.0, None, None, False
        a = math.atan2(vy, vx)                 # direction of travel; wheels point along it
        if a > math.pi / 2:                    # fold into +-90 deg, drive backwards instead
            a, s = a - math.pi, -s
        elif a < -math.pi / 2:
            a, s = a + math.pi, -s
        lim = min(self.max_front, self.max_rear)   # alpha = beta, so the tighter limit applies
        a = clamp(a, -lim, lim)
        s = clamp(s, -self.vmax, self.vmax)
        return s, s, a, a, False

    def _joint(self, now):
        jc = self.joint_cmd
        if jc is None:
            return 0.0, 0.0, None, None, True
        (a, b, vl, vr), t = jc
        ta = clamp(math.radians(a), -self.max_front, self.max_front)
        tb = clamp(math.radians(b), -self.max_rear, self.max_rear)
        if not (0.0 <= now - t <= self.cmd_timeout):
            return 0.0, 0.0, ta, tb, True      # timeout: speeds -> 0, angles hold
        return clamp(vl, -self.vmax, self.vmax), clamp(vr, -self.vmax, self.vmax), ta, tb, False

    # ------------------------------------------------------------------ output shaping
    def _slew(self, dt):
        step = self.slew * dt
        self.alpha += clamp(self.t_alpha - self.alpha, -step, step)
        self.beta += clamp(self.t_beta - self.beta, -step, step)

    def _ramp(self, tvl, tvr, dt, hard):
        tvl = clamp(tvl, -self.vmax, self.vmax)
        tvr = clamp(tvr, -self.vmax, self.vmax)
        if hard or self.accel <= 0.0:
            self.vl, self.vr = tvl, tvr
            return
        step = self.accel * dt
        self.vl += clamp(tvl - self.vl, -step, step)
        self.vr += clamp(tvr - self.vr, -step, step)

    def _publish(self):
        a = self.front_dir * self.alpha
        b = self.rear_dir * self.beta
        if self.sim:
            self.pub_front.publish(Float64(data=float(a)))
            self.pub_rear.publish(Float64(data=float(b)))
            self.pub_fl.publish(Float64(data=float(self.vl / self.r)))
            self.pub_fr.publish(Float64(data=float(self.vr / self.r)))
        else:
            self.pub_front.publish(Float32(data=float(math.degrees(a))))
            self.pub_rear.publish(Float32(data=float(math.degrees(b))))
            self.pub_fl.publish(Float32(data=float(self.vl)))
            self.pub_fr.publish(Float32(data=float(self.vr)))

    def _note_source(self, name):
        if name != self.last_source:
            self.get_logger().info(f'Active source: {name} (drive mode {self.active})')
            self.last_source = name

    # ------------------------------------------------------------------ main loop
    def _cycle(self):
        now = self.now_s()
        dt = clamp(now - self.last_t, 0.0, 0.1)
        self.last_t = now

        # CRAB / JOINT only exist in MANUAL with the e-stop clear
        if self.estop or self.control_mode != 'MANUAL':
            self.requested = 'NORMAL'

        # 1) e-stop: speeds to zero instantly, angles hold; recentre once released
        if self.estop:
            if self.active != 'NORMAL' or abs(self.alpha) > EPS or abs(self.beta) > EPS:
                self.pending_recentre = True
            self.active, self.stage = 'NORMAL', None
            self.vl = self.vr = 0.0
            self._publish()
            self._note_source('e-stop')
            return

        # 2) mode change (or post-e-stop): stop, recentre alpha = beta = 0, wait the settle time
        if self.stage is None and (self.requested != self.active or self.pending_recentre):
            self.stage, self.pending_recentre = 'slew', False
            self.get_logger().info(f'Recentring steering ({self.active} -> {self.requested})')
        if self.stage is not None:
            self.vl = self.vr = 0.0
            self.t_alpha = self.t_beta = 0.0
            self._slew(dt)
            if self.stage == 'slew' and abs(self.alpha) < EPS and abs(self.beta) < EPS:
                self.stage, self.stage_end = 'wait', now + self.settle
            elif self.stage == 'wait' and now >= self.stage_end:
                self.active, self.stage = self.requested, None
                self.reached_since = now - self.settle      # allow CRAB to start straight away
                self.joint_cmd = None                       # never replay an old jog command
                self.get_logger().info(f'Drive mode: {self.active}')
            self._publish()
            self._note_source('recentring')
            return

        # 3) normal operation
        src, tw = self._select_twist(now)
        if self.active == 'JOINT':
            src, tw = 'joint', self.joint_cmd
            tvl, tvr, ta, tb, hard = self._joint(now)
        elif self.active == 'CRAB':
            tvl, tvr, ta, tb, hard = self._crab(tw)
        else:
            tvl, tvr, ta, tb, hard = self._normal(tw)
        prev_ta, prev_tb = self.t_alpha, self.t_beta
        if ta is not None:
            self.t_alpha = ta
        if tb is not None:
            self.t_beta = tb
        if self.active == 'CRAB' and max(abs(self.t_alpha - prev_ta), abs(self.t_beta - prev_tb)) > self.deadband:
            self.reached_since = None
        self._slew(dt)

        # CRAB: no driving until both steering angles are on target for steer_settle_time
        if self.active == 'CRAB' and ta is not None:
            err = max(abs(self.t_alpha - self.alpha), abs(self.t_beta - self.beta))
            if err <= self.deadband:
                if self.reached_since is None:
                    self.reached_since = now
            else:
                self.reached_since = None
            if self.reached_since is None or now - self.reached_since < self.settle:
                tvl = tvr = 0.0
                hard = True

        self._ramp(tvl, tvr, dt, hard)
        self._publish()
        self._note_source(src if tw is not None else f'{src} (silent)')


def main(args=None):
    rclpy.init(args=args)
    node = BaseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()