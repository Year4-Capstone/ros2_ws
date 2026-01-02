#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
import yaml
import time
import os
import cv2
import numpy as np


class CoverageWaypointGenerator(Node):
    def __init__(self):
        super().__init__('coverage_waypoint_generator')

        self.declare_parameter('map_yaml')
        self.declare_parameter('spacing', 0.75)

        self.map_yaml = self.get_parameter('map_yaml').value
        self.spacing = float(self.get_parameter('spacing').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.init_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 1
        )

        self.client = ActionClient(self, FollowWaypoints, '/follow_waypoints')

        self.wait_for_amcl()
        self.publish_initial_pose()
        self.wait_for_tf()
        self.client.wait_for_server()

        waypoints = self.generate_waypoints()
        self.send_waypoints(waypoints)

    def wait_for_amcl(self):
        self.get_logger().info('Waiting for AMCL...')
        time.sleep(3.0)

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.orientation.w = 1.0
        self.init_pub.publish(msg)
        self.get_logger().info('Initial pose published')
        time.sleep(1.0)

    def wait_for_tf(self):
        self.get_logger().info('Waiting for map->base_footprint TF...')
        while rclpy.ok():
            try:
                self.tf_buffer.lookup_transform(
                    'map', 'base_footprint', rclpy.time.Time()
                )
                return
            except Exception:
                time.sleep(0.2)

    def generate_waypoints(self):
        with open(self.map_yaml, 'r') as f:
            cfg = yaml.safe_load(f)

        res = cfg['resolution']
        ox, oy, _ = cfg['origin']

        map_dir = os.path.dirname(self.map_yaml)
        img_path = os.path.join(map_dir, cfg['image'])

        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        h, w = img.shape

        waypoints = []

        for y in range(0, h, int(self.spacing / res)):
            for x in range(0, w, int(self.spacing / res)):
                if img[y, x] > 250:
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.pose.position.x = ox + x * res
                    pose.pose.position.y = oy + y * res
                    pose.pose.orientation.w = 1.0
                    waypoints.append(pose)

        self.get_logger().info(f'{len(waypoints)} waypoints generated')
        return waypoints

    def send_waypoints(self, waypoints):
        goal = FollowWaypoints.Goal()
        goal.poses = waypoints
        self.client.send_goal_async(goal)


def main():
    rclpy.init()
    node = CoverageWaypointGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
