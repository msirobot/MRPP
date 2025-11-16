#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
import yaml
import os

class WaypointManager:
    def __init__(self):
        self.waypoints = []
        self.current_index = 0
        self.load_waypoints()

    def load_waypoints(self):
        """从YAML文件加载航点"""
        try:
            waypoints_path = os.path.join(os.path.dirname(__file__), 
                                        '../../config/waypoints.yaml')
            with open(waypoints_path, 'r') as file:
                config = yaml.safe_load(file)
                waypoint_data = config.get('waypoints', [])
                for wp in waypoint_data:
                    point = Point()
                    point.x = wp['x']
                    point.y = wp['y']
                    point.z = 0.0
                    self.waypoints.append(point)
            rospy.loginfo("Loaded %d waypoints", len(self.waypoints))
        except Exception as e:
            rospy.logerr("Failed to load waypoints: %s", e)

    def get_current_waypoint(self):
        """获取当前航点"""
        if not self.waypoints:
            return None
        return self.waypoints[self.current_index]

    def advance_waypoint(self):
        """前进到下一个航点"""
        if not self.waypoints:
            return
        self.current_index = (self.current_index + 1) % len(self.waypoints)

    def is_waypoint_reached(self, position, threshold=0.1):
        """检查是否到达当前航点"""
        current_wp = self.get_current_waypoint()
        if current_wp is None:
            return False
        distance = ((position[0] - current_wp.x)**2 + (position[1] - current_wp.y)**2)**0.5
        return distance < threshold