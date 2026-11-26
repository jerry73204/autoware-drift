"""The race planner as an Autoware node: L1 (re-planning) and L2 (tracking) behind Autoware's own
seams, a first step of the race stack (docs/race-stack-design.md): a wrapper around the
planner's JSON-lines process (race_planner.serve), which runs in the planner's own Python
(CasADi, IPOPT, HPIPM) while this node runs in ROS's.

    in   /localization/kinematic_state        nav_msgs/Odometry (base_link: the rear axle; its
                                              twist the body's)
         /vehicle/status/steering_status      autoware_vehicle_msgs/SteeringReport
         /api/operation_mode/state            autoware_adapi_v1_msgs/OperationModeState
    out  /control/trajectory_follower/control_cmd  autoware_control_msgs/Control (the trajectory
                                              follower's topic: Autoware's gate takes it from
                                              there; trajectory_follower_mode:=none)
         /planning/trajectory                 autoware_planning_msgs/Trajectory (the plan the
                                              car is on, from the car on; planning_setting:=race
                                              leaves the topic to this node)
         /race/status                         std_msgs/String (JSON: the planner's tick record)

One state in, one command out: the command is stamped with the state's stamp, so a simulator
in lockstep (FastSlowRR's isaac/bridge.py) can wait for the answer to the state it sent.

Until Autoware is in autonomous mode the planner is not started: the node answers each state
with a hold (no acceleration, the wheel straight) and a trajectory straight ahead at rest, so
Autoware can engage. Then the launch: straight on at launch_accel up to launch_speed (the planner
starts from a rolling car). From there every state goes to the planner, its first plan solved
then (not under the budget).

Parameters (ROS): rp_python, rp_root (the planner's Python and its tree), init (the serve init
record as JSON: spec, track or map, car, budget, det, ...; see race_planner/serve.py) or init_file
(a file of it), wait_engage
(default true; false starts the planner on the first state), launch_speed and launch_accel (the
launch, below), log_file (each tick's record, as /race/status, in a JSON-lines file).
"""
import json
import math
import os
import subprocess
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_control_msgs.msg import Control
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from autoware_vehicle_msgs.msg import SteeringReport
from nav_msgs.msg import Odometry
from std_msgs.msg import String

HOLD_TRAJ_M, HOLD_TRAJ_DS = 100.0, 2.0      # the trajectory straight ahead before the start


class Serve:
    """race_planner.serve as a child process, one JSON line each way."""

    def __init__(self, python, root, init):
        env = {k: v for k, v in os.environ.items() if k != "PYTHONPATH"}   # not ROS's packages
        env.setdefault("OPENBLAS_NUM_THREADS", "1")
        env.setdefault("OMP_NUM_THREADS", "1")
        self.p = subprocess.Popen([python, "-m", "race_planner.serve"], cwd=root, env=env,
                                  stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True,
                                  bufsize=1)
        self.ready = self.ask({"init": init})
        if not self.ready.get("ready"):
            raise RuntimeError(f"race_planner.serve: {self.ready}")

    def ask(self, msg):
        self.p.stdin.write(json.dumps(msg) + "\n")
        self.p.stdin.flush()
        line = self.p.stdout.readline()
        if not line:
            raise RuntimeError(f"race_planner.serve ended (exit {self.p.poll()})")
        return json.loads(line)

    def close(self):
        try:
            done = self.ask({"stop": True})
        except (RuntimeError, BrokenPipeError, ValueError):
            done = None
        self.p.wait(timeout=10)
        return done


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class RacePlanner(Node):
    def __init__(self):
        super().__init__("race_planner")
        here = os.path.expanduser("~/repos/autoware-drift/src/planning/race_planner")
        self.declare_parameter("rp_root", here)
        self.declare_parameter("rp_python", os.path.join(here, ".venv/bin/python"))
        self.declare_parameter("init", "{}")
        self.declare_parameter("init_file", "")
        self.declare_parameter("wait_engage", True)
        # the launch: the planner starts from a rolling car (its first plan from rest is not a
        # number, IPOPT's Invalid_Number_Detected: the model divides by the speed), so from rest
        # the car is driven straight at launch_accel until launch_speed, then handed over
        self.declare_parameter("launch_speed", 5.0)
        self.declare_parameter("launch_accel", 4.0)
        # every tick's planner record (/race/status's) appended to this file too ("": none)
        self.declare_parameter("log_file", "")
        lf = self.get_parameter("log_file").value
        self.log = open(lf, "w") if lf else None
        self.launch_speed = float(self.get_parameter("launch_speed").value)
        self.launch_accel = float(self.get_parameter("launch_accel").value)
        root = self.get_parameter("rp_root").value
        f = self.get_parameter("init_file").value
        self.init = json.load(open(f)) if f else json.loads(self.get_parameter("init").value)
        self.wait_engage = bool(self.get_parameter("wait_engage").value)
        self.serve = Serve(self.get_parameter("rp_python").value, root, self.init)
        self.get_logger().info(f"race_planner.serve ready: {self.serve.ready}")
        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.p_cmd = self.create_publisher(Control, "/control/trajectory_follower/control_cmd", q)
        self.p_traj = self.create_publisher(Trajectory, "/planning/trajectory", q)
        self.p_status = self.create_publisher(String, "/race/status", q)
        self.create_subscription(Odometry, "/localization/kinematic_state", self.on_state, q)
        self.create_subscription(SteeringReport, "/vehicle/status/steering_status", self.on_steer, q)
        self.create_subscription(OperationModeState, "/api/operation_mode/state", self.on_mode, q)
        self.delta = 0.0
        self.autonomous = not self.wait_engage
        self.started = False
        self.plans = {}            # plan_id -> its message (x, y, psi, v, t, s)
        self.plan_id = None
        self.ticks = 0
        self.lock = threading.Lock()

    def on_steer(self, m):
        self.delta = float(m.steering_tire_angle)

    def on_mode(self, m):
        auto = (m.mode == OperationModeState.AUTONOMOUS and m.is_autoware_control_enabled)
        if auto and not self.autonomous:
            self.get_logger().info("autonomous: the planner starts")
        self.autonomous = self.autonomous or auto

    def on_state(self, m):
        with self.lock:
            stamp = m.header.stamp
            t = stamp.sec + 1e-9 * stamp.nanosec
            p, tw = m.pose.pose, m.twist.twist
            yaw = _yaw(p.orientation)
            if not self.autonomous:
                self.publish(stamp, 0.0, 0.0, 0.0)
                self.publish_hold(m.header, p.position.x, p.position.y, p.position.z, yaw)
                return
            if not self.started and tw.linear.x < self.launch_speed:
                self.publish(stamp, self.launch_accel, 0.0, self.launch_speed)
                self.publish_hold(m.header, p.position.x, p.position.y, p.position.z, yaw,
                                  self.launch_speed)
                return
            state = {"t": t, "x": p.position.x, "y": p.position.y, "yaw": yaw,
                     "vx": tw.linear.x, "vy": tw.linear.y, "r": tw.angular.z,
                     "delta": self.delta}
            out = self.serve.ask({"state": state})
            self.started = True
            if "plan" in out:
                self.plans[out["plan_id"]] = out["plan"]
                for k in [k for k in self.plans if k < out["plan_id"] - 2]:
                    del self.plans[k]
            self.plan_id = out.get("plan_id", self.plan_id)
            self.publish(stamp, out["accel"], out["tire_angle"], out.get("v_plan") or 0.0)
            self.publish_plan(m.header, out.get("s", 0.0), p.position.z)
            self.ticks += 1
            rec = json.dumps({k: v for k, v in out.items() if k != "plan"} | {"t": t}, default=float)
            self.p_status.publish(String(data=rec))
            if self.log is not None:
                self.log.write(rec + "\n")

    def publish(self, stamp, accel, tire_angle, v):
        c = Control()
        c.stamp = c.lateral.stamp = c.longitudinal.stamp = stamp
        c.control_time = c.lateral.control_time = c.longitudinal.control_time = stamp
        c.lateral.steering_tire_angle = float(tire_angle)
        c.longitudinal.velocity = float(v)
        c.longitudinal.acceleration = float(accel)
        c.longitudinal.is_defined_acceleration = True
        self.p_cmd.publish(c)

    def _point(self, x, y, z, yaw, v, t):
        pt = TrajectoryPoint()
        pt.pose.position.x, pt.pose.position.y, pt.pose.position.z = float(x), float(y), float(z)
        pt.pose.orientation.z, pt.pose.orientation.w = math.sin(0.5 * yaw), math.cos(0.5 * yaw)
        pt.longitudinal_velocity_mps = float(v)
        sec = max(float(t), 0.0)
        pt.time_from_start.sec = int(sec)
        pt.time_from_start.nanosec = int((sec - int(sec)) * 1e9)
        return pt

    def publish_hold(self, header, x, y, z, yaw, v=0.0):
        tr = Trajectory()
        tr.header.stamp, tr.header.frame_id = header.stamp, "map"
        n = int(HOLD_TRAJ_M / HOLD_TRAJ_DS) + 1
        c, s = math.cos(yaw), math.sin(yaw)
        tr.points = [self._point(x + k * HOLD_TRAJ_DS * c, y + k * HOLD_TRAJ_DS * s, z, yaw, v, 0.0)
                     for k in range(n)]
        self.p_traj.publish(tr)

    def publish_plan(self, header, s_car, z):
        """The plan the car is on, from the car's arclength on (the planner's nodes)."""
        pl = self.plans.get(self.plan_id)
        if pl is None or "x" not in pl:
            return
        tr = Trajectory()
        tr.header.stamp, tr.header.frame_id = header.stamp, "map"
        s = pl["s"]
        k0 = max(0, next((k for k, v in enumerate(s) if v >= s_car), len(s) - 1) - 1)
        t0 = pl["t"][k0]
        tr.points = [self._point(pl["x"][k], pl["y"][k], z, pl["psi"][k], pl["v"][k], pl["t"][k] - t0)
                     for k in range(k0, len(s))]
        self.p_traj.publish(tr)

    def close(self):
        if self.log is not None:
            self.log.close()
        done = self.serve.close()
        self.get_logger().info(f"race_planner.serve done: {done}")


def main():
    rclpy.init()
    node = RacePlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
