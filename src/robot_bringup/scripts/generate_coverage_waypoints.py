#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.action import ActionClient

from nav2_msgs.action import FollowWaypoints
from lifecycle_msgs.srv import GetState

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped

import tf2_ros
import yaml
import numpy as np
import os
import time


class CoverageWaypointGenerator(Node):

    def __init__(self):
        super().__init__('coverage_waypoint_generator')

        # -----------------------
        # Parameters
        # -----------------------
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('spacing', 0.75)
        self.declare_parameter('free_pixel_min', 250)
        self.declare_parameter('send_waypoints_cap', 500)
        self.declare_parameter('base_frame', 'base_footprint')

        self.map_yaml = self.get_parameter('map_yaml').get_parameter_value().string_value
        self.spacing = float(self.get_parameter('spacing').value)
        self.free_pixel_min = int(self.get_parameter('free_pixel_min').value)
        self.send_cap = int(self.get_parameter('send_waypoints_cap').value)
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        self.get_logger().info(f"map_yaml: {self.map_yaml}")
        self.get_logger().info(f"spacing: {self.spacing}")
        self.get_logger().info(f"free_pixel_min: {self.free_pixel_min}")
        self.get_logger().info(f"send_waypoints_cap: {self.send_cap}")
        self.get_logger().info(f"base_frame: {self.base_frame}")
        self.get_logger().info("Coverage waypoint generator initialized")

        # -----------------------
        # TF
        # -----------------------
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # -----------------------
        # Marker publisher (latched)
        # -----------------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/coverage_waypoints', qos)

        # -----------------------
        # Action client
        # -----------------------
        self.follow_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')

        # -----------------------
        # Lifecycle check (waypoint_follower)
        # -----------------------
        self.wp_state_client = self.create_client(GetState, '/waypoint_follower/get_state')

        # -----------------------
        # State
        # -----------------------
        self.map_loaded = False
        self.tf_ready = False
        self.action_ready = False
        self.wp_active = False

        self.waypoints = []
        self.last_publish_time = 0.0
        self.sent_action = False

        self._throttle_last = {}

        # -----------------------
        # Timers
        # -----------------------
        self.create_timer(0.5, self.status_timer)
        self.create_timer(5.0, self.republish_timer)

    # ============================================================
    # Small throttle helper (works on Jazzy)
    # ============================================================
    def log_throttle(self, level: str, key: str, msg: str, period_s: float = 5.0):
        now = time.time()
        last = self._throttle_last.get(key, 0.0)
        if (now - last) < period_s:
            return
        self._throttle_last[key] = now

        if level == 'info':
            self.get_logger().info(msg)
        elif level == 'warn':
            self.get_logger().warn(msg)
        else:
            self.get_logger().error(msg)

    # ============================================================
    # Main gating loop
    # ============================================================
    def status_timer(self):
        # 0) Map YAML exists + readable
        self.check_map()

        # 1) Action server exists
        self.check_action_server()

        # 2) Waypoint follower ACTIVE (optional but useful)
        self.check_waypoint_follower_state()

        # 3) TF map->base exists
        self.check_tf()

        # Log gates (throttled)
        if not self.map_loaded:
            self.log_throttle('warn', 'gate_map', "GATE: waiting for map YAML", 2.0)
            return
        if not self.action_ready:
            self.log_throttle('warn', 'gate_action', "GATE: waiting for /follow_waypoints action server", 2.0)
            return
        if not self.wp_active:
            self.log_throttle('warn', 'gate_wp_active', "GATE: waiting for waypoint_follower ACTIVE", 2.0)
            return
        if not self.tf_ready:
            self.log_throttle('warn', 'gate_tf', f"GATE: waiting for TF map -> {self.base_frame}", 2.0)
            return

        # 4) Generate once
        if not self.waypoints:
            self.generate_waypoints()

        # 5) Send once
        if self.waypoints and not self.sent_action:
            self.send_waypoints()

    # ============================================================
    # Checks
    # ============================================================
    def check_map(self):
        if self.map_loaded:
            return

        if not self.map_yaml or not os.path.exists(self.map_yaml):
            self.log_throttle('warn', 'map_yaml_missing', f"Waiting for map_yaml to exist: {self.map_yaml}", 5.0)
            return

        try:
            with open(self.map_yaml, 'r') as f:
                yaml.safe_load(f)
            self.map_loaded = True
            self.get_logger().info("Map YAML readable")
        except Exception as e:
            self.log_throttle('error', 'map_yaml_read_fail', f"Failed reading map YAML: {e}", 5.0)

    def check_action_server(self):
        if self.action_ready:
            return

        if self.follow_client.wait_for_server(timeout_sec=0.0):
            self.action_ready = True
            self.get_logger().info("FollowWaypoints action server available")

    def check_waypoint_follower_state(self):
        if self.wp_active:
            return

        if not self.wp_state_client.service_is_ready():
            self.log_throttle('warn', 'wp_state_srv_wait', "Waiting for /waypoint_follower/get_state service", 5.0)
            return

        req = GetState.Request()
        future = self.wp_state_client.call_async(req)
        future.add_done_callback(self.wp_state_cb)

    def wp_state_cb(self, future):
        try:
            res = future.result()
            label = res.current_state.label
            if label == 'active':
                if not self.wp_active:
                    self.wp_active = True
                    self.get_logger().info("waypoint_follower is ACTIVE")
            else:
                self.log_throttle('warn', 'wp_not_active', f"waypoint_follower state: {label}", 5.0)
        except Exception as e:
            self.log_throttle('warn', 'wp_state_fail', f"waypoint_follower get_state failed: {e}", 5.0)

    def check_tf(self):
        if self.tf_ready or not self.map_loaded:
            return

        try:
            self.tf_buffer.lookup_transform(
                'map',
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            self.tf_ready = True
            self.get_logger().info(f"TF ready: map -> {self.base_frame}")
        except tf2_ros.LookupException as e:
            self.log_throttle('warn', 'tf_lookup', f'TF not ready (missing frame): {e}', 2.0)
        except tf2_ros.ConnectivityException as e:
            self.log_throttle('warn', 'tf_conn', f'TF connectivity issue: {e}', 2.0)
        except tf2_ros.ExtrapolationException as e:
            self.log_throttle('warn', 'tf_extrap', f'TF extrapolation issue (timing): {e}', 2.0)
        except Exception as e:
            self.log_throttle('warn', 'tf_other', f'TF lookup failed: {type(e).__name__}: {e}', 2.0)

    # ============================================================
    # Waypoint generation
    # ============================================================
    def generate_waypoints(self):
        self.get_logger().info("Generating coverage waypoints")

        try:
            with open(self.map_yaml, 'r') as f:
                map_yaml = yaml.safe_load(f)

            resolution = float(map_yaml['resolution'])
            origin = map_yaml['origin']
            image_path = map_yaml['image']
            if not os.path.isabs(image_path):
                image_path = os.path.join(os.path.dirname(self.map_yaml), image_path)

            self.get_logger().info(f"Map image: {image_path}")
            self.get_logger().info(f"resolution: {resolution}, origin: {origin}")

            image = self.load_pgm(image_path)

            # PGM values: 0=black, 255=white.
            # Your map_server treats "free_thresh" etc; here we do a simple "white-ish means free".
            free = image >= self.free_pixel_min

            h, w = free.shape
            step = int(self.spacing / resolution)
            if step <= 0:
                self.get_logger().error(f"Computed step={step} (spacing/resolution). Check spacing/resolution.")
                return

            self.get_logger().info(f"grid: {w}x{h}, step_px: {step}")

            count = 0
            for y in range(0, h, step):
                for x in range(0, w, step):
                    if free[y, x]:
                        wx = origin[0] + x * resolution
                        wy = origin[1] + y * resolution

                        pose = PoseStamped()
                        pose.header.frame_id = 'map'
                        pose.pose.position.x = float(wx)
                        pose.pose.position.y = float(wy)
                        pose.pose.orientation.w = 1.0

                        self.waypoints.append(pose)
                        count += 1
                        if count >= self.send_cap:
                            break
                if count >= self.send_cap:
                    break

            self.get_logger().info(f"Generated {len(self.waypoints)} waypoints")

            if not self.waypoints:
                self.get_logger().error("No waypoints generated (free space test produced 0).")
                self.get_logger().error("Try lowering free_pixel_min (e.g. 200) or confirm your map encoding.")
                return

            self.publish_markers()

        except Exception as e:
            self.get_logger().error(f"Waypoint generation failed: {type(e).__name__}: {e}")

    # ============================================================
    # Publishing
    # ============================================================
    def publish_markers(self):
        ma = MarkerArray()
        now_msg = self.get_clock().now().to_msg()

        for i, pose in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now_msg
            m.ns = 'coverage'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose = pose.pose
            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = 0.15
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 1.0
            ma.markers.append(m)

        self.marker_pub.publish(ma)
        self.last_publish_time = time.time()
        self.get_logger().info(f"Published MarkerArray on /coverage_waypoints (count={len(ma.markers)})")

    def republish_timer(self):
        # republish every 10s after first publish so RViz always catches it
        if self.waypoints and (time.time() - self.last_publish_time) > 10.0:
            self.get_logger().warn("Republishing waypoint markers (RViz latch safety)")
            self.publish_markers()

    # ============================================================
    # Action
    # ============================================================
    def send_waypoints(self):
        if self.sent_action:
            return

        # Action server already gated, but keep it safe
        if not self.follow_client.wait_for_server(timeout_sec=1.0):
            self.log_throttle('warn', 'follow_wait', "Waiting for /follow_waypoints action server (send)", 2.0)
            return

        goal = FollowWaypoints.Goal()
        goal.poses = self.waypoints

        self.get_logger().info(f"Sending FollowWaypoints goal (poses={len(goal.poses)})")
        fut = self.follow_client.send_goal_async(goal)
        fut.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, fut):
        try:
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().error("FollowWaypoints goal rejected")
                return

            self.get_logger().info("FollowWaypoints goal accepted")
            self.sent_action = True

            result_fut = goal_handle.get_result_async()
            result_fut.add_done_callback(self.result_cb)

        except Exception as e:
            self.get_logger().error(f"Goal response callback failed: {type(e).__name__}: {e}")

    def result_cb(self, fut):
        try:
            res = fut.result()
            self.get_logger().info(f"FollowWaypoints result status={res.status}")
            result = res.result
            self.get_logger().info(f"missed_waypoints={result.missed_waypoints}")
        except Exception as e:
            self.get_logger().error(f"Result callback failed: {type(e).__name__}: {e}")

    # ============================================================
    # Utilities
    # ============================================================
    def load_pgm(self, path):
        if not os.path.exists(path):
            raise FileNotFoundError(path)

        with open(path, 'rb') as f:
            data = f.read()

        if not data.startswith(b'P5'):
            raise RuntimeError("PGM is not P5")

        i = 2
        tokens = []
        while len(tokens) < 3:
            while i < len(data) and data[i] in b' \t\r\n':
                i += 1
            if i < len(data) and data[i] == ord('#'):
                while i < len(data) and data[i] != ord('\n'):
                    i += 1
                continue
            start = i
            while i < len(data) and data[i] not in b' \t\r\n':
                i += 1
            tokens.append(data[start:i])

        w = int(tokens[0])
        h = int(tokens[1])
        maxval = int(tokens[2])
        if maxval != 255:
            raise RuntimeError(f"Unsupported maxval {maxval}")

        while i < len(data) and data[i] in b' \t\r\n':
            i += 1

        pixels = np.frombuffer(data[i:], dtype=np.uint8, count=w * h)
        if pixels.size != w * h:
            raise RuntimeError("PGM pixel data truncated")

        return pixels.reshape((h, w))


def main():
    rclpy.init()
    node = CoverageWaypointGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
