#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Point
import yaml
import os

class MotionPlanner:
    def __init__(self, robot_poses):
        self.robot_poses = robot_poses
        self.waypoints = []
        self.current_waypoint_index = 0
        self.waypoint_threshold = 0.5  # 到达航点的距离阈值
        
        # 加载航点配置
        self.load_waypoints()

    def load_waypoints(self):
        """加载航点配置"""
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
                    self.waypoints.append(point)
                    
            rospy.loginfo("Loaded %d waypoints", len(self.waypoints))
        except Exception as e:
            rospy.logwarn("Using default waypoints: %s", e)
            # 默认航点
            self.waypoints = [
                Point(2.0, 0.0, 0.0),
                Point(4.0, 2.0, 0.0),
                Point(2.0, 4.0, 0.0),
                Point(0.0, 2.0, 0.0)
            ]

    def update(self):
        """更新运动规划，返回目标位置"""
        if not self.waypoints:
            return self.get_formation_positions()
            
        current_waypoint = self.waypoints[self.current_waypoint_index]
        
        # 检查是否到达当前航点
        leader_pos = list(self.robot_poses.values())[0][:2]  # 第一个机器人的位置
        distance = np.sqrt(
            (leader_pos[0] - current_waypoint.x)**2 + 
            (leader_pos[1] - current_waypoint.y)**2
        )
        
        if distance < self.waypoint_threshold:
            self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
            rospy.loginfo("Reached waypoint %d, moving to next", self.current_waypoint_index)
        
        return self.get_formation_positions()

    def get_formation_positions(self):
        """根据当前航点生成编队位置"""
        if not self.waypoints:
            return {}
            
        current_wp = self.waypoints[self.current_waypoint_index]
        robot_names = list(self.robot_poses.keys())
        
        target_poses = {}
        
        # 根据编队类型计算目标位置
        formation_type = rospy.get_param('~formation_type', 'line')
        
        if formation_type == 'line':
            # 直线编队
            for i, name in enumerate(robot_names):
                offset = i * 0.5  # 0.5米间隔
                target_poses[name] = [
                    current_wp.x + offset,
                    current_wp.y,
                    0.0
                ]
                
        elif formation_type == 'triangle':
            # 三角形编队
            if len(robot_names) >= 3:
                target_poses[robot_names[0]] = [current_wp.x, current_wp.y, 0.0]
                target_poses[robot_names[1]] = [current_wp.x - 0.5, current_wp.y - 0.5, 0.0]
                target_poses[robot_names[2]] = [current_wp.x + 0.5, current_wp.y - 0.5, 0.0]
                # 处理额外的机器人
                for i in range(3, len(robot_names)):
                    target_poses[robot_names[i]] = [current_wp.x, current_wp.y - i*0.5, 0.0]
                
        elif formation_type == 'circle':
            # 圆形编队
            radius = 1.0
            for i, name in enumerate(robot_names):
                angle = 2 * np.pi * i / len(robot_names)
                target_poses[name] = [
                    current_wp.x + radius * np.cos(angle),
                    current_wp.y + radius * np.sin(angle),
                    angle + np.pi/2
                ]
                
        else:  # 默认直线编队
            for i, name in enumerate(robot_names):
                target_poses[name] = [current_wp.x + i*0.5, current_wp.y, 0.0]
        
        return target_poses