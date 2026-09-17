#!/usr/bin/env python3
"""
Formation Control Node for PX4 + ROS 2 (3-drone triangle formation)

ARCHITECTURE CHANGE FROM THE PREVIOUS VERSION:

The previous version maintained its own drifting position setpoint
(self.setpoint += ALPHA * u every tick) AND separately published a
raw velocity feed-forward (a straight copy of the neighbor's
velocity, un-scaled, un-damped) at the same time. Sending both a
position AND a velocity setpoint to PX4 simultaneously, where the
velocity term had no error-correction or damping behaviour at all,
created a cascaded-loop-with-lag situation -- which is why it went
unstable at surprisingly low gains (0.07-0.1) instead of the ~0.3
a simple single-loop system would tolerate.

THIS VERSION instead sends ONLY a velocity command each tick,
computed directly as a proper PD (proportional-derivative) law:

    u_i = kp * (p_j - p_i - p_i_des + p_j_des) + kv * (v_j - v_i)

This is the double-integrator control law from Oh, Park & Ahn
(2015), Eq. (17) -- the kp term pulls the drone toward the correct
relative position, and the kv term is REAL damping: it resists
the drone's own velocity relative to its neighbor, actively
counteracting overshoot, rather than just copying the neighbor's
speed unconditionally.

There is no separate internal integrator state to accumulate lag
or wind up -- every tick recomputes a fresh command straight from
current sensor data. PX4's own internal controller (which already
integrates velocity into position, using proper attitude control
underneath) does the actual "turning velocity into motion" job --
we don't need to duplicate that ourselves.

REQUIREMENTS:
    - ROS 2 (Humble, inside your Docker container)
    - px4_msgs built in your ROS 2 workspace
    - PX4 SITL running with multiple instances (one per drone)

USAGE (three separate terminals, one per drone):
    python3 formation_control.py --id 0
    python3 formation_control.py --id 1
    python3 formation_control.py --id 2

TOPIC NAMESPACING (confirmed from your `ros2 topic list | grep fmu`):
    instance 0  -> /fmu/...        (NO namespace prefix)
    instance N  -> /px4_N/fmu/...  (N > 0)

VERSIONED TOPIC NAMES (confirmed from your setup):
    vehicle_local_position_v1, vehicle_status_v4
    If you rebuild against a different PX4 version, re-check these
    with `ros2 topic list | grep fmu`.
"""

import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

# ---------------------------------------------------------------------------
# CONFIGURATION
# ---------------------------------------------------------------------------

def get_namespace(px4_instance: int) -> str:
    if px4_instance == 0:
        return ""
    return f"/px4_{px4_instance}"


LOCAL_POSITION_TOPIC = "vehicle_local_position_v1"
VEHICLE_STATUS_TOPIC = "vehicle_status_v4"

# Desired triangle formation, offsets from drone 0, in WORLD frame
# (X=East, Y=North), metres. Z is NED-down (negative = higher).
DESIRED_OFFSETS = {
    0: (0.0, 0.0, -5.0),
    1: (-5.0, -5.0, -5.0),
    2: (5.0, -5.0, -5.0),
}

# Each drone's spawn offset in the shared WORLD frame -- must match
# whatever you passed via PX4_GZ_MODEL_POSE at spawn time.
SPAWN_WORLD_OFFSET = {
    0: (0.0, 0.0),
    1: (5.0, 0.0),
    2: (-5.0, 0.0),
}

NEIGHBORS = {
    0: [],
    1: [0],
    2: [0],
}

# --- PD gains for followers (Eq. 17: u = kp*position_error + kv*velocity_error) ---
# Start here and tune. If you see oscillation, INCREASE Kv first
# (more damping) before touching Kp. If tracking feels sluggish
# once stable, increase Kp a little at a time.
KP = 1.0
KV = 1.5    

# --- Leader (drone 0) waypoint pursuit ---
LEADER_WAYPOINTS = [
    (0.0, 0.0, -5.0),
    (0.0, 10.0, -5.0),
]
WAYPOINT_ARRIVAL_RADIUS = 0.5   # metres
LEADER_KP = 0.5                 # proportional gain, pursuit velocity = LEADER_KP * error
LEADER_MAX_SPEED = 2.0          # m/s cap on leader's commanded velocity

CONTROL_PERIOD_SEC = 0.1  # 10Hz, matches PX4's >=2Hz offboard requirement

# ---------------------------------------------------------------------------


class FormationControlNode(Node):
    def __init__(self, drone_id: int):
        super().__init__(f"formation_control_drone_{drone_id}")

        self.drone_id = drone_id
        self.mav_sys_id = drone_id + 1  # PX4 sim convention: MAV_SYS_ID = instance + 1
        self.namespace = get_namespace(drone_id)
        self.neighbors = NEIGHBORS[drone_id]
        self.desired_offset = DESIRED_OFFSETS[drone_id]

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- Publishers ---
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, f"{self.namespace}/fmu/in/offboard_control_mode", qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f"{self.namespace}/fmu/in/trajectory_setpoint", qos_profile)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, f"{self.namespace}/fmu/in/vehicle_command", qos_profile)

        # --- Own state subscribers ---
        self.own_position_sub = self.create_subscription(
            VehicleLocalPosition, f"{self.namespace}/fmu/out/{LOCAL_POSITION_TOPIC}",
            self.own_position_callback, qos_profile)
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, f"{self.namespace}/fmu/out/{VEHICLE_STATUS_TOPIC}",
            self.vehicle_status_callback, qos_profile)

        # --- Neighbor subscribers ---
        self.neighbor_positions = {}   # neighbor_id -> (world_x, world_y, z) or None
        self.neighbor_velocities = {}  # neighbor_id -> (vx, vy, vz), OWN local frame, or None
        for n_id in self.neighbors:
            self.neighbor_positions[n_id] = None
            self.neighbor_velocities[n_id] = None
            n_namespace = get_namespace(n_id)
            self.create_subscription(
                VehicleLocalPosition, f"{n_namespace}/fmu/out/{LOCAL_POSITION_TOPIC}",
                self.make_neighbor_callback(n_id), qos_profile)

        # --- Internal state ---
        self.own_position = None   # (world_x, world_y, z)
        self.own_velocity = None   # (vx, vy, vz), PX4's own local NED frame -- no swap needed
        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter = 0
        self.waypoint_index = 0

        self.timer = self.create_timer(CONTROL_PERIOD_SEC, self.control_loop)

        self.get_logger().info(
            f"Formation control node started for drone {drone_id} "
            f"(namespace={self.namespace or '<none>'}, "
            f"mav_sys_id={self.mav_sys_id}, neighbors={self.neighbors})"
        )

    # -----------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------

    def own_position_callback(self, msg: VehicleLocalPosition):
        sx, sy = SPAWN_WORLD_OFFSET[self.drone_id]
        # NED/ENU North-East swap, confirmed empirically (see original
        # comment in the previous version of this file for the
        # measurement that established this). Applied to BOTH position
        # and velocity so every axis of the control law stays in one
        # consistent frame -- mixing swapped position with unswapped
        # velocity was the bug causing the diagonal drift.
        world_x = sx + msg.y
        world_y = sy + msg.x
        self.own_position = (world_x, world_y, msg.z)
        world_vx = msg.vy
        world_vy = msg.vx
        self.own_velocity = (world_vx, world_vy, msg.vz)

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_status = msg

    def make_neighbor_callback(self, neighbor_id: int):
        def callback(msg: VehicleLocalPosition):
            sx, sy = SPAWN_WORLD_OFFSET[neighbor_id]
            world_x = sx + msg.y
            world_y = sy + msg.x
            self.neighbor_positions[neighbor_id] = (world_x, world_y, msg.z)
            # Same swap applied to velocity as position -- both must
            # live in the same (world) frame for pos_error and
            # vel_error to combine correctly on matching axes.
            world_vx = msg.vy
            world_vy = msg.vx
            self.neighbor_velocities[neighbor_id] = (world_vx, world_vy, msg.vz)
        return callback

    # -----------------------------------------------------------------
    # Core control law
    # -----------------------------------------------------------------

    def compute_follower_velocity_command(self):
        """
        u_i = kp * sum[(p_j - p_i) - (p_j_des - p_i_des)]
            + kv * sum(v_j - v_i)

        Returns a WORLD-frame [vx, vy, vz] velocity command, or None
        if data isn't ready yet. This is computed fresh every tick --
        no accumulated/integrated state, so no windup, no lag buildup.
        """
        if self.own_position is None or self.own_velocity is None:
            return None

        u = [0.0, 0.0, 0.0]
        for n_id in self.neighbors:
            p_j = self.neighbor_positions.get(n_id)
            v_j = self.neighbor_velocities.get(n_id)
            if p_j is None or v_j is None:
                return None

            p_i_des = self.desired_offset
            p_j_des = DESIRED_OFFSETS[n_id]

            for axis in range(3):
                pos_error = (self.own_position[axis] - p_j[axis]) - (p_i_des[axis] - p_j_des[axis])
                # Both own and neighbor velocities are now in WORLD
                # frame (see own_position_callback / make_neighbor_callback),
                # matching pos_error's frame -- axis 0 of both terms
                # now genuinely refers to the same physical direction.
                vel_error = self.own_velocity[axis] - v_j[axis]
                u[axis] += -KP * pos_error - KV * vel_error

        return u

    def compute_leader_velocity_command(self):
        """
        Simple proportional pursuit of the current waypoint, speed-
        capped. Returns a WORLD-frame [vx, vy, vz] velocity command.
        """
        if self.own_position is None:
            return None

        target = LEADER_WAYPOINTS[self.waypoint_index]
        error = [target[axis] - self.own_position[axis] for axis in range(3)]
        dist = math.sqrt(sum(e * e for e in error))

        if dist < WAYPOINT_ARRIVAL_RADIUS and len(LEADER_WAYPOINTS) > 1:
            self.waypoint_index = (self.waypoint_index + 1) % len(LEADER_WAYPOINTS)

        velocity = [LEADER_KP * e for e in error]
        speed = math.sqrt(sum(v * v for v in velocity))
        if speed > LEADER_MAX_SPEED and speed > 0:
            scale = LEADER_MAX_SPEED / speed
            velocity = [v * scale for v in velocity]

        return velocity

    def control_loop(self):
        self.publish_offboard_control_mode()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.neighbors:
            u_world = self.compute_follower_velocity_command()
        else:
            u_world = self.compute_leader_velocity_command()

        if u_world is not None:
            # World frame (X=East,Y=North) -> PX4 local NED (x=North,
            # y=East): swap back before publishing.
            u_local = [u_world[1], u_world[0], u_world[2]]
            self.publish_trajectory_setpoint(u_local)

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    # -----------------------------------------------------------------
    # Publishing helpers
    # -----------------------------------------------------------------

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True   # velocity-only control now, for everyone
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, velocity_local):
        msg = TrajectorySetpoint()
        # NaN position tells PX4 "ignore this field, use velocity only"
        # -- avoids the dual position+velocity ambiguity from before.
        msg.position = [float("nan"), float("nan"), float("nan")]
        msg.velocity = [float(velocity_local[0]), float(velocity_local[1]), float(velocity_local[2])]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = self.mav_sys_id
        msg.target_component = 1
        msg.source_system = self.mav_sys_id
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info(f"Drone {self.drone_id}: arm command sent")

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info(f"Drone {self.drone_id}: switching to offboard mode")


def main():
    parser = argparse.ArgumentParser(description="Formation control node for one drone")
    parser.add_argument("--id", type=int, required=True, choices=[0, 1, 2],
                         help="PX4 instance number, matching `px4 -i N`")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = FormationControlNode(drone_id=args.id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
