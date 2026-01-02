#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

import yaml
import os
import cv2
import numpy as np


class CoverageWaypointGenerator(Node):

    def __init__(self):
        super().__init__('coverage_waypoint_generator')

        # ---------------- Parameters ----------------
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('spacing', 0.75)
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('max_waypoints', 5000)

        self.map_yaml   = self.get_parameter('map_yaml').value
        self.spacing    = float(self.get_parameter('spacing').value)
        self.base_frame = self.get_parameter('base_frame').value
        self.max_wps    = int(self.get_parameter('max_waypoints').value)

        if not self.map_yaml or not os.path.exists(self.map_yaml):
            self.get_logger().fatal(f'Invalid map_yaml: {self.map_yaml}')
            raise RuntimeError('map_yaml missing')

        if self.spacing <= 0.0:
            self.get_logger().fatal('spacing must be > 0')
            raise RuntimeError('invalid spacing')

        self.get_logger().info('Coverage waypoint generator starting')

        # ---------------- TF ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------- Action ----------------
        self.client = ActionClient(self, FollowWaypoints, '/follow_waypoints')

        # ---------------- State ----------------
        self.nav2_ready = False
        self.tf_ready = False
        self.executed = False

        # ---------------- Timer (core fix) ----------------
        self.timer = self.create_timer(0.5, self.tick)

    # ==========================================================
    # Main state machine
    # ==========================================================

    def tick(self):
        if self.executed:
            return

        # 1. Wait for Nav2 action server
        if not self.nav2_ready:
            if self.client.wait_for_server(timeout_sec=0.0):
                self.nav2_ready = True
                self.get_logger().info('FollowWaypoints action server available')
            else:
                return

        # 2. Wait for localization (map -> base TF)
        if not self.tf_ready:
            try:
                self.tf_buffer.lookup_transform(
                    'map',
                    self.base_frame,
                    rclpy.time.Time()
                )
                self.tf_ready = True
                self.get_logger().info('Localization detected (map TF available)')
            except Exception:
                return

        # 3. Execute coverage exactly once
        self.get_logger().info('Generating and sending coverage waypoints')
        waypoints = self.generate_waypoints()
        self.send_waypoints(waypoints)

        self.executed = True
        self.timer.cancel()
        self.get_logger().info('Coverage node completed startup sequence')

    # ==========================================================
    # Waypoint generation
    # ==========================================================

    def generate_waypoints(self):
        with open(self.map_yaml, 'r') as f:
            cfg = yaml.safe_load(f)

        res = cfg['resolution']
        ox, oy, _ = cfg['origin']

        img_path = os.path.join(
            os.path.dirname(self.map_yaml),
            cfg['image']
        )

        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().fatal('Failed to load map image')
            raise RuntimeError('map image missing')

        h, w = img.shape
        step_px = int(self.spacing / res)

        if step_px <= 0:
            self.get_logger().fatal('spacing too small for map resolution')
            raise RuntimeError('invalid spacing')

        waypoints = []

        for y in range(0, h, step_px):
            for x in range(0, w, step_px):
                if img[y, x] > 250:
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.pose.position.x = ox + x * res
                    pose.pose.position.y = oy + y * res
                    pose.pose.orientation.w = 1.0
                    waypoints.append(pose)

        self.get_logger().info(f'{len(waypoints)} waypoints generated')

        if not waypoints:
            raise RuntimeError('No waypoints generated')

        if len(waypoints) > self.max_wps:
            raise RuntimeError('Too many waypoints generated')

        return waypoints

    # ==========================================================
    # Action handling
    # ==========================================================

    def send_waypoints(self, waypoints):
        goal = FollowWaypoints.Goal()
        goal.poses = waypoints

        self.get_logger().info('Sending FollowWaypoints goal')
        fut = self.client.send_goal_async(goal)
        fut.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, fut):
        goal_handle = fut.result()
        if not goal_handle.accepted:
            self.get_logger().error('FollowWaypoints goal rejected')
            return

        self.get_logger().info('FollowWaypoints goal accepted')
        result_fut = goal_handle.get_result_async()
        result_fut.add_done_callback(self.result_cb)

    def result_cb(self, fut):
        result = fut.result().result
        self.get_logger().info(
            f'FollowWaypoints completed. missed_waypoints={result.missed_waypoints}'
        )


def main():
    rclpy.init()
    node = CoverageWaypointGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
