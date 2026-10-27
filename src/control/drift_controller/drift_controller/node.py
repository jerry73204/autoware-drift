"""drift_controller's ROS node: between Autoware's command gate and the car.

    /control/command/control_cmd         Autoware's Control (in)
    /control/command/drift/control_cmd   the car's Control (out): Autoware's, passed through,
                                         or the drift law's while a drift runs
    /drift/state                         a line of JSON per phase change and once a second

The vehicle bridge takes the car's Control from the second topic (aib's ego config
`control_topic`, `ego_drift_mustang.yaml`).

What the law needs, from what Autoware already has:
- pose and yaw: `/localization/kinematic_state` (the EKF);
- yaw rate: the IMU (`/sensing/imu/imu_data`, Autoware's corrected one);
- Autoware's trajectory: `/planning/scenario_planning/trajectory`, for the hand-back: the car's
  lateral and heading error from its nearest point and a pure-pursuit tire angle onto it
  (the law's REJOIN), so Autoware takes the car back within its transition manager's limits.
- Autoware's emergency: `/api/fail_safe/mrm_state`. While an MRM runs no drift starts; one under
  way is aborted to RECOVER (steering along the car's travel, so it does not spin under
  braking), braking at least as hard as Autoware's command asks; after it, Autoware's own
  command (its emergency stop) passes through.
- speed and sideslip: the direction and length of the car's travel between consecutive NDT
  poses (`/localization/pose_estimator/pose_with_covariance`, 10 Hz) against its heading.
  The EKF's own twist has no lateral velocity: its model drives along the heading, so in a
  drift its velocity says sideslip 0. NDT's poses are scan matches, independent of that.

The law (`law.py`) runs on a timer at `rate_hz` with the latest state; outside a drift the node
forwards Autoware's command as it arrives.
"""
import json
import math

import rclpy
import yaml
from autoware_adapi_v1_msgs.msg import MrmState
from autoware_control_msgs.msg import Control
from autoware_planning_msgs.msg import Trajectory
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from .law import DriftController, DriftPoint, Params, State


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def load_points(path):
    doc = yaml.safe_load(open(path))
    return [DriftPoint(p["x"], p["y"], p["radius"], p.get("name", ""),
                       math.radians(p.get("angle_deg", 0.0))) for p in doc["points"]]


WHEEL_BASE = 2.886  # drift_mustang's; the pure pursuit's only geometry


def path_errors(points, x, y, yaw, speed):
    """(lateral error, heading error, pure-pursuit tire angle, speed) against a trajectory's
    poses [(x, y, yaw, v)], from the nearest point; None when there is no trajectory."""
    if len(points) < 2:
        return None, None, None, None
    i = min(range(len(points)), key=lambda k: (points[k][0] - x) ** 2 + (points[k][1] - y) ** 2)
    px, py, pyaw, pv = points[i]
    lat = -(x - px) * math.sin(pyaw) + (y - py) * math.cos(pyaw)
    head = math.remainder(yaw - pyaw, math.tau)
    look = max(6.0, 0.6 * speed)
    j, run = i, 0.0
    while j + 1 < len(points) and run < look:
        run += math.hypot(points[j + 1][0] - points[j][0], points[j + 1][1] - points[j][1])
        j += 1
    tx, ty = points[j][0], points[j][1]
    alpha = math.remainder(math.atan2(ty - y, tx - x) - yaw, math.tau)
    steer = math.atan2(2 * WHEEL_BASE * math.sin(alpha), max(math.hypot(tx - x, ty - y), 1.0))
    return lat, head, steer, pv


class TravelEstimate:
    """Speed and course from consecutive scan-matched poses."""

    def __init__(self, min_dt=0.05, max_dt=0.5):
        self.last, self.min_dt, self.max_dt = None, min_dt, max_dt
        self.speed, self.course = 0.0, None

    def add(self, t, x, y):
        if self.last is not None:
            t0, x0, y0 = self.last
            dt = t - t0
            if dt < self.min_dt:
                return
            if dt <= self.max_dt:
                self.speed = math.hypot(x - x0, y - y0) / dt
                self.course = math.atan2(y - y0, x - x0) if self.speed > 1.0 else None
        self.last = (t, x, y)


class DriftNode(Node):
    def __init__(self):
        super().__init__("drift_controller")
        points_file = self.declare_parameter("points_file", "").value
        rate = self.declare_parameter("rate_hz", 100.0).value
        overrides = json.loads(self.declare_parameter("params", "").value or "{}")
        overrides.setdefault("repeat", self.declare_parameter("repeat", True).value)
        self.points = load_points(points_file) if points_file else []
        self.law = DriftController(self.points, Params(**overrides))
        self.dt = 1.0 / rate
        reliable = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(Control, "/control/command/drift/control_cmd", reliable)
        self.state_pub = self.create_publisher(String, "/drift/state", 10)
        self.create_subscription(Control, "/control/command/control_cmd", self.on_autoware,
                                 reliable)
        self.create_subscription(Odometry, "/localization/kinematic_state", self.on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped,
                                 "/localization/pose_estimator/pose_with_covariance",
                                 self.on_ndt, 10)
        self.create_subscription(Imu, "/sensing/imu/imu_data", self.on_imu,
                                 qos_profile_sensor_data)
        self.create_subscription(MrmState, "/api/fail_safe/mrm_state", self.on_mrm, 10)
        self.create_subscription(Trajectory, "/planning/scenario_planning/trajectory",
                                 self.on_trajectory, 1)
        self.trajectory = []
        self.mrm = False
        self.create_timer(self.dt, self.tick)
        self.create_timer(1.0, self.report)
        self.autoware = None
        self.pose = None          # (x, y, yaw)
        self.ekf_speed = 0.0
        self.yaw_rate = 0.0
        self.travel = TravelEstimate()
        self.last_phase = self.law.phase
        self.last_cmd = None
        self.get_logger().info(f"{len(self.points)} drift point(s) from {points_file or '-'}, "
                               f"law at {rate:.0f} Hz")

    def on_autoware(self, msg):
        self.autoware = msg
        if not self.law_overrides():
            self.pub.publish(msg)

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        self.ekf_speed = msg.twist.twist.linear.x

    def on_ndt(self, msg):
        t = msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec
        self.travel.add(t, msg.pose.pose.position.x, msg.pose.pose.position.y)

    def on_mrm(self, msg):
        mrm = msg.state not in (MrmState.UNKNOWN, MrmState.NORMAL)
        if mrm and not self.mrm:
            self.get_logger().warn(f"Autoware MRM (state {msg.state}, behavior {msg.behavior}): "
                                   "no drift starts; one under way recovers")
            if self.pose is not None:
                self.law.abort(self.state(), "Autoware MRM")
        self.mrm = mrm
        self.law.armed = not mrm

    def on_trajectory(self, msg):
        self.trajectory = [(p.pose.position.x, p.pose.position.y, yaw_of(p.pose.orientation),
                            p.longitudinal_velocity_mps) for p in msg.points]

    def on_imu(self, msg):
        self.yaw_rate = msg.angular_velocity.z

    def law_overrides(self):
        return self.last_cmd is not None and self.last_cmd.override

    def state(self):
        x, y, yaw = self.pose
        speed = self.travel.speed if self.travel.course is not None else self.ekf_speed
        beta = 0.0
        if self.travel.course is not None:
            beta = math.remainder(self.travel.course - yaw, math.tau)
        lat, head, steer, pv = path_errors(self.trajectory, x, y, yaw, speed)
        return State(x, y, yaw, speed * math.cos(beta), speed * math.sin(beta), self.yaw_rate,
                     lat, head, steer, pv)

    def tick(self):
        if self.pose is None:
            return
        s = self.state()
        cmd = self.law.step(s, self.dt)
        self.last_cmd = cmd
        if self.law.phase is not self.last_phase:
            self.last_phase = self.law.phase
            self.report(s)
        if not cmd.override:
            return
        msg = Control()
        now = self.get_clock().now().to_msg()
        msg.stamp = msg.lateral.stamp = msg.longitudinal.stamp = now
        msg.lateral.steering_tire_angle = float(cmd.steer)
        accel = cmd.accel
        if self.mrm and self.autoware is not None:  # brake at least as hard as Autoware asks
            accel = min(accel, self.autoware.longitudinal.acceleration)
        msg.longitudinal.acceleration = float(accel)
        msg.longitudinal.velocity = float(s.speed + 10.0 if accel > 0 else 0.0)
        self.pub.publish(msg)

    def report(self, s=None):
        if s is None:
            if self.pose is None:
                return
            s = self.state()
        self.state_pub.publish(String(data=json.dumps({
            "phase": self.law.phase.value, "point": self.law.active.name if self.law.active else None,
            "next": self.law.next_index, "x": round(s.x, 2), "y": round(s.y, 2),
            "speed": round(s.speed, 2), "beta_deg": round(math.degrees(s.beta), 2),
            "r": round(s.r, 3), "log": self.law.log[-1] if self.law.log else None})))


def main():
    rclpy.init()
    node = DriftNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
