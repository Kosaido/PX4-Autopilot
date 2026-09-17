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

DEPLOYMENT MODES (see DEPLOYMENT flag in CONFIGURATION below):
    "SITL" -- multi-instance PX4 in one container/Gazebo world.
              Position comes from each vehicle's local NED position
              plus a hardcoded SPAWN_WORLD_OFFSET (matching
              PX4_GZ_MODEL_POSE). Namespace prefixing and
              MAV_SYS_ID follow PX4's SITL instance convention.
    "REAL" -- one Raspberry Pi per physical drone, each running its
              own single PX4 + Micro XRCE-DDS Agent. Topics are
              always bare (no /px4_N prefix -- nothing to
              disambiguate on a single-vehicle Pi). Position comes
              from GPS (VehicleGlobalPosition) converted to a shared
              North-East-Down frame relative to a fixed, pre-measured
              origin (FIXED_ORIGIN_LAT/LON/ALT) -- set these before
              flying. MAV_SYS_ID must match each vehicle's actual
              configured value (REAL_MAV_SYS_ID), not the SITL
              instance+1 convention -- verify with `param show
              MAV_SYS_ID` in each vehicle's own PX4 shell.

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
    VehicleGlobalPosition,
    VehicleStatus,
)

# ---------------------------------------------------------------------------
# CONFIGURATION
# ---------------------------------------------------------------------------

# Flip this when moving from SITL to real hardware. See the class
# docstring notes throughout for exactly what changes between modes.
DEPLOYMENT = "SITL"  # "SITL" or "REAL"

# --- REAL-mode only: fixed GPS origin for the flying field ---
# Every drone converts its own GPS fix into North-East-Down metres
# relative to THIS SAME point, which is what gives all drones a
# shared coordinate frame on real hardware (SITL gets this for free
# from SPAWN_WORLD_OFFSET instead). Set these once, e.g. from a phone
# GPS reading taken standing at your field, before a REAL-mode flight.
FIXED_ORIGIN_LAT = None   # e.g. -6.892489
FIXED_ORIGIN_LON = None   # e.g. 107.610426
FIXED_ORIGIN_ALT = None   # metres AMSL; 0.0 is fine if unknown -- only affects absolute altitude, not formation shape

# --- REAL-mode only: each drone's actual configured MAV_SYS_ID ---
# PX4's SITL convention (mav_sys_id = instance+1) does NOT apply on
# real hardware -- a freshly-flashed vehicle defaults to 1 unless you
# deliberately changed it. VERIFY each vehicle's real value with
# `param show MAV_SYS_ID` in its own PX4 shell before flying, and
# update this dict to match. Get this wrong and that drone will
# silently ignore every arm/mode command sent to it.
REAL_MAV_SYS_ID = {0: 1, 1: 1, 2: 1}


def gps_to_ned(lat, lon, alt, ref_lat, ref_lon, ref_alt):
    """
    Flat-earth (equirectangular) approximation, accurate to well
    under a centimetre of error at formation-flying scales (tens of
    metres). Returns (north, east, down) in metres relative to the
    reference point.
    """
    R = 6371000.0  # metres
    north = (lat - ref_lat) * (math.pi / 180.0) * R
    east = (lon - ref_lon) * (math.pi / 180.0) * R * math.cos(math.radians(ref_lat))
    down = -(alt - ref_alt)
    return north, east, down


def get_namespace(px4_instance: int) -> str:
    if DEPLOYMENT == "REAL":
        # Each Pi bridges only its own single PX4 instance -- there's
        # nothing to disambiguate, so topics are always bare.
        return ""
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
KP = 0.6
KV = 0.8

# --- Leader (drone 0) waypoint pursuit ---
LEADER_WAYPOINTS = [
    (0.0, 0.0, -5.0),
    (10.0, 0.0, -5.0),
]
WAYPOINT_ARRIVAL_RADIUS = 0.5   # metres
LEADER_KP = 0.5                 # proportional gain, pursuit velocity = LEADER_KP * error
LEADER_MAX_SPEED = 2.0          # m/s cap on leader's commanded velocity

# --- Safe takeoff sequence ---
# Every drone climbs straight up, at its own current horizontal
# position, to this altitude BEFORE any formation/pursuit logic
# runs at all -- no horizontal movement happens until altitude is
# reached. TAKEOFF_ALTITUDE matches the formation's own cruise
# altitude (-5.0 = 5m up in NED), so there's no second climb/descent
# needed after switching into FORMATION state.
TAKEOFF_ALTITUDE = -5.0          # metres, NED (negative = up)
TAKEOFF_ARRIVAL_THRESHOLD = 0.3  # metres -- how close counts as "arrived"
TAKEOFF_KP = 0.5                 # proportional gain on altitude error
TAKEOFF_MAX_SPEED = 1.0          # m/s cap on vertical climb speed (deliberately gentle)

CONTROL_PERIOD_SEC = 0.1  # 10Hz, matches PX4's >=2Hz offboard requirement

# ---------------------------------------------------------------------------


class FormationControlNode(Node):
    def __init__(self, drone_id: int):
        super().__init__(f"formation_control_drone_{drone_id}")

        self.drone_id = drone_id
        if DEPLOYMENT == "REAL":
            self.mav_sys_id = REAL_MAV_SYS_ID[drone_id]  # verify against actual hardware, see config above
        else:
            self.mav_sys_id = drone_id + 1  # PX4 SITL convention: MAV_SYS_ID = instance + 1
        self.namespace = get_namespace(drone_id)
        self.neighbors = NEIGHBORS[drone_id]
        self.desired_offset = DESIRED_OFFSETS[drone_id]

        if DEPLOYMENT == "REAL" and None in (FIXED_ORIGIN_LAT, FIXED_ORIGIN_LON, FIXED_ORIGIN_ALT):
            raise RuntimeError(
                "DEPLOYMENT is 'REAL' but FIXED_ORIGIN_LAT/LON/ALT are not set. "
                "Set them to your flying field's coordinates before running on real hardware."
            )

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
        # Local position always feeds velocity; in SITL it ALSO feeds
        # position (via SPAWN_WORLD_OFFSET). In REAL mode, position
        # instead comes from GPS (see own_global_position_callback)
        # since there's no shared Gazebo/SITL frame to lean on.
        self.own_position_sub = self.create_subscription(
            VehicleLocalPosition, f"{self.namespace}/fmu/out/{LOCAL_POSITION_TOPIC}",
            self.own_local_position_callback, qos_profile)
        if DEPLOYMENT == "REAL":
            self.own_global_position_sub = self.create_subscription(
                VehicleGlobalPosition, f"{self.namespace}/fmu/out/vehicle_global_position",
                self.own_global_position_callback, qos_profile)
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, f"{self.namespace}/fmu/out/{VEHICLE_STATUS_TOPIC}",
            self.vehicle_status_callback, qos_profile)

        # --- Neighbor subscribers ---
        self.neighbor_positions = {}   # neighbor_id -> (world_x, world_y, z) or None
        self.neighbor_velocities = {}  # neighbor_id -> (vx, vy, vz), WORLD frame, or None
        for n_id in self.neighbors:
            self.neighbor_positions[n_id] = None
            self.neighbor_velocities[n_id] = None
            n_namespace = get_namespace(n_id)
            self.create_subscription(
                VehicleLocalPosition, f"{n_namespace}/fmu/out/{LOCAL_POSITION_TOPIC}",
                self.make_neighbor_local_callback(n_id), qos_profile)
            if DEPLOYMENT == "REAL":
                self.create_subscription(
                    VehicleGlobalPosition, f"{n_namespace}/fmu/out/vehicle_global_position",
                    self.make_neighbor_global_callback(n_id), qos_profile)

        # --- Internal state ---
        self.own_position = None   # (world_x, world_y, z)
        self.own_velocity = None   # (vx, vy, vz), WORLD frame
        self.vehicle_status = VehicleStatus()
        self.offboard_setpoint_counter = 0
        self.waypoint_index = 0

        # Safety state machine: every drone climbs vertically first,
        # in place, before any formation/pursuit logic engages at
        # all. See compute_takeoff_velocity_command / control_loop.
        self.flight_state = "TAKEOFF"  # -> "FORMATION" once altitude reached

        self.timer = self.create_timer(CONTROL_PERIOD_SEC, self.control_loop)

        self.get_logger().info(
            f"Formation control node started for drone {drone_id} "
            f"(namespace={self.namespace or '<none>'}, "
            f"mav_sys_id={self.mav_sys_id}, neighbors={self.neighbors})"
        )

    # -----------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------

    def own_local_position_callback(self, msg: VehicleLocalPosition):
        # Velocity: PX4 local NED is always (x=North, y=East), but our
        # DESIRED_OFFSETS/world convention is (X=East, Y=North) -- this
        # swap is just that fixed axis-order difference, true in BOTH
        # SITL and REAL (nothing to do with Gazebo specifically).
        world_vx = msg.vy
        world_vy = msg.vx
        self.own_velocity = (world_vx, world_vy, msg.vz)

        if DEPLOYMENT == "SITL":
            # Position: SITL has no GPS, so the shared frame comes
            # from SPAWN_WORLD_OFFSET (matching PX4_GZ_MODEL_POSE)
            # plus the same North/East swap as velocity above.
            sx, sy = SPAWN_WORLD_OFFSET[self.drone_id]
            world_x = sx + msg.y
            world_y = sy + msg.x
            self.own_position = (world_x, world_y, msg.z)
        # In REAL mode, position instead comes from
        # own_global_position_callback (GPS) -- this callback only
        # supplies velocity there.

    def own_global_position_callback(self, msg: VehicleGlobalPosition):
        # REAL mode only. GPS already gives every drone an absolute,
        # shared reference -- no per-vehicle spawn offset needed.
        #
        # GPS isn't instant: right after power-on the module needs
        # time to acquire a fix, and PX4's EKF needs a few consistent
        # seconds before trusting it. lat_lon_valid/alt_valid are
        # PX4's own signal that the estimate is actually usable --
        # ignoring them would let a stale/pre-lock position through
        # as if it were real, bypassing every "wait until data's
        # ready" safeguard elsewhere in this file (self.own_position
        # would go non-None immediately with garbage in it).
        if not (msg.lat_lon_valid and msg.alt_valid):
            return

        north, east, down = gps_to_ned(
            msg.lat, msg.lon, msg.alt,
            FIXED_ORIGIN_LAT, FIXED_ORIGIN_LON, FIXED_ORIGIN_ALT,
        )
        self.own_position = (east, north, down)  # (X=East, Y=North, Z=down), matching DESIRED_OFFSETS

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_status = msg

    def make_neighbor_local_callback(self, neighbor_id: int):
        def callback(msg: VehicleLocalPosition):
            world_vx = msg.vy
            world_vy = msg.vx
            self.neighbor_velocities[neighbor_id] = (world_vx, world_vy, msg.vz)

            if DEPLOYMENT == "SITL":
                sx, sy = SPAWN_WORLD_OFFSET[neighbor_id]
                world_x = sx + msg.y
                world_y = sy + msg.x
                self.neighbor_positions[neighbor_id] = (world_x, world_y, msg.z)
        return callback

    def make_neighbor_global_callback(self, neighbor_id: int):
        def callback(msg: VehicleGlobalPosition):
            if not (msg.lat_lon_valid and msg.alt_valid):
                return
            north, east, down = gps_to_ned(
                msg.lat, msg.lon, msg.alt,
                FIXED_ORIGIN_LAT, FIXED_ORIGIN_LON, FIXED_ORIGIN_ALT,
            )
            self.neighbor_positions[neighbor_id] = (east, north, down)
        return callback

    # -----------------------------------------------------------------
    # Core control law
    # -----------------------------------------------------------------

    def compute_takeoff_velocity_command(self):
        """
        Climb straight up (no horizontal motion at all) toward
        TAKEOFF_ALTITUDE, at whatever horizontal position the drone
        happens to be at. Returns a WORLD-frame [vx, vy, vz]
        velocity command -- a safe zero-velocity hover if position
        data isn't ready yet (see compute_follower_velocity_command
        for why this must never be None).

        Sets self.flight_state = "FORMATION" once altitude is
        reached, so this only needs to run once per drone at startup.
        """
        if self.own_position is None:
            return [0.0, 0.0, 0.0]

        altitude_error = TAKEOFF_ALTITUDE - self.own_position[2]  # NED: negative = up

        if abs(altitude_error) < TAKEOFF_ARRIVAL_THRESHOLD:
            self.flight_state = "FORMATION"
            self.get_logger().info(
                f"Drone {self.drone_id}: takeoff altitude reached, "
                f"switching to FORMATION"
            )
            return [0.0, 0.0, 0.0]

        vz = TAKEOFF_KP * altitude_error
        vz = max(-TAKEOFF_MAX_SPEED, min(TAKEOFF_MAX_SPEED, vz))

        # Deliberately zero horizontal velocity -- climb straight up,
        # don't drift toward the formation offset until altitude-safe.
        return [0.0, 0.0, vz]

    def compute_follower_velocity_command(self):
        """
        u_i = kp * sum[(p_j - p_i) - (p_j_des - p_i_des)]
            + kv * sum(v_j - v_i)

        Returns a WORLD-frame [vx, vy, vz] velocity command. This is
        computed fresh every tick -- no accumulated/integrated state,
        so no windup, no lag buildup.

        IMPORTANT: this used to return None whenever own/neighbor
        data wasn't ready yet, which meant control_loop skipped
        publishing a trajectory setpoint entirely for that tick. PX4
        requires a continuous >=2Hz offboard setpoint stream or it
        trips a setpoint-timeout failsafe and drops out of offboard
        mode -- so any gap in neighbor data (DDS discovery delay, a
        dropped BEST_EFFORT packet, network jitter) could silently
        kick a follower out of offboard control, while the leader
        (which never depends on external data) never hits this at
        all. Returning a safe zero-velocity hover instead of None
        keeps the heartbeat alive until fresh data arrives.
        """
        if self.own_position is None or self.own_velocity is None:
            return [0.0, 0.0, 0.0]

        u = [0.0, 0.0, 0.0]
        for n_id in self.neighbors:
            p_j = self.neighbor_positions.get(n_id)
            v_j = self.neighbor_velocities.get(n_id)
            if p_j is None or v_j is None:
                return [0.0, 0.0, 0.0]

            p_i_des = self.desired_offset
            p_j_des = DESIRED_OFFSETS[n_id]

            for axis in range(3):
                pos_error = (self.own_position[axis] - p_j[axis]) - (p_i_des[axis] - p_j_des[axis])
                # Both own and neighbor velocities are now in WORLD
                # frame (see own_local_position_callback / own_global_position_callback),
                # matching pos_error's frame -- axis 0 of both terms
                # now genuinely refers to the same physical direction.
                vel_error = self.own_velocity[axis] - v_j[axis]
                u[axis] += -KP * pos_error - KV * vel_error

        return u

    def compute_leader_velocity_command(self):
        """
        Simple proportional pursuit of the current waypoint, speed-
        capped. Returns a WORLD-frame [vx, vy, vz] velocity command
        -- a safe zero-velocity hover if position data isn't ready
        yet, for the same reason as compute_follower_velocity_command.
        """
        if self.own_position is None:
            return [0.0, 0.0, 0.0]

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

        if self.flight_state == "TAKEOFF":
            u_world = self.compute_takeoff_velocity_command()
        elif self.neighbors:
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


def run(drone_id: int):
    """
    Entry point called by each per-drone launcher file
    (drone0_main.py, drone1_main.py, drone2_main.py). Kept as a
    plain function (not tied to argparse) so each launcher can just
    hardcode its own id -- no command-line arguments needed when
    deployed on a Raspberry Pi.
    """
    rclpy.init()
    node = FormationControlNode(drone_id=drone_id)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    """
    Kept for convenience when testing on a dev machine where all
    three drones' code lives in one place (e.g. your Docker
    container) -- lets you still run `python3 formation_control_core.py
    --id 0` directly instead of using a launcher file.
    """
    parser = argparse.ArgumentParser(description="Formation control node for one drone")
    parser.add_argument("--id", type=int, required=True, choices=[0, 1, 2],
                         help="PX4 instance number, matching `px4 -i N`")
    args, _ = parser.parse_known_args()
    run(drone_id=args.id)


if __name__ == "__main__":
    main()