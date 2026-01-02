#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import yaml
import os
import math
import cv2
import numpy as np


class MapDiffNode(Node):
    def __init__(self):
        super().__init__('map_diff_node')

        self.declare_parameter('baseline_map')
        self.declare_parameter('min_hits', 20)
        self.declare_parameter('cluster_radius', 0.5)
        self.declare_parameter('max_range', 6.0)

        self.map_yaml = self.get_parameter('baseline_map').value
        self.min_hits = int(self.get_parameter('min_hits').value)
        self.cluster_radius = float(self.get_parameter('cluster_radius').value)
        self.max_range = float(self.get_parameter('max_range').value)

        self.load_map()

        self.hits = {}

        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_cb, 10
        )
        self.pub = self.create_publisher(
            MarkerArray, '/map_diff_markers', 1
        )

    def load_map(self):
        with open(self.map_yaml, 'r') as f:
            cfg = yaml.safe_load(f)

        self.res = cfg['resolution']
        self.ox, self.oy, _ = cfg['origin']

        map_dir = os.path.dirname(self.map_yaml)
        img_path = os.path.join(map_dir, cfg['image'])

        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise RuntimeError(f'Failed to load map image {img_path}')

        self.occ = img < 200

    def scan_cb(self, scan):
        a = scan.angle_min
        for r in scan.ranges:
            if math.isinf(r) or r > self.max_range:
                a += scan.angle_increment
                continue

            x = r * math.cos(a)
            y = r * math.sin(a)

            mx = int((x - self.ox) / self.res)
            my = int((y - self.oy) / self.res)

            if (
                0 <= my < self.occ.shape[0]
                and 0 <= mx < self.occ.shape[1]
                and not self.occ[my, mx]
            ):
                key = (round(x, 2), round(y, 2))
                self.hits[key] = self.hits.get(key, 0) + 1

            a += scan.angle_increment

        self.publish_markers()

    def publish_markers(self):
        arr = MarkerArray()
        idx = 0

        for (x, y), hits in self.hits.items():
            if hits < self.min_hits:
                continue

            if self.is_wall_like(x, y):
                continue

            m = Marker()
            m.header.frame_id = 'map'
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.id = idx
            m.scale.x = m.scale.y = m.scale.z = 0.25
            m.color.r = 1.0
            m.color.a = 1.0
            m.pose.position.x = x
            m.pose.position.y = y
            arr.markers.append(m)
            idx += 1

        self.pub.publish(arr)

    def is_wall_like(self, x, y):
        for (ox, oy), _ in self.hits.items():
            if math.hypot(ox - x, oy - y) < self.cluster_radius:
                return True
        return False


def main():
    rclpy.init()
    node = MapDiffNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
