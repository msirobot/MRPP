#!/usr/bin/env python3

import rospy
import numpy as np
from math import atan2, sqrt

class FormationController:
    def __init__(self, robot_poses):
        self.robot_poses = robot_poses
        self.kp_linear = 0.5   # 线性比例增益
        self.kp_angular = 1.0  # 角度比例增益
        self.max_linear_speed = 0.5
        self.max_angular_speed = 1.0

    def update(self, target_poses, formation_type='line'):
        """更新编队控制，返回速度命令"""
        velocity_commands = {}
        
        for robot_name, current_pose in self.robot_poses.items():
            if robot_name not in target_poses:
                continue
                
            target_pose = target_poses[robot_name]
            
            # 计算位置误差
            dx = target_pose[0] - current_pose[0]
            dy = target_pose[1] - current_pose[1]
            distance_error = sqrt(dx*dx + dy*dy)
            
            # 计算角度误差
            desired_theta = atan2(dy, dx)
            theta_error = self.normalize_angle(desired_theta - current_pose[2])
            
            # PID控制
            linear_vel = self.kp_linear * distance_error
            angular_vel = self.kp_angular * theta_error
            
            # 速度限制
            linear_vel = np.clip(linear_vel, -self.max_linear_speed, self.max_linear_speed)
            angular_vel = np.clip(angular_vel, -self.max_angular_speed, self.max_angular_speed)
            
            velocity_commands[robot_name] = (linear_vel, angular_vel)
            
        return velocity_commands

    def normalize_angle(self, angle):
        """标准化角度到[-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def calculate_formation_offsets(self, formation_type, num_robots):
        """计算编队偏移量"""
        if formation_type == 'line':
            # 水平直线编队
            return [(i * 0.5, 0.0, 0.0) for i in range(num_robots)]
            
        elif formation_type == 'triangle':
            # 三角形编队
            if num_robots >= 3:
                return [
                    (0.0, 0.0, 0.0),      # 领航者
                    (-0.5, -0.5, 0.0),    # 左后方
                    (0.5, -0.5, 0.0)      # 右后方
                ]
            else:
                return self.calculate_formation_offsets('line', num_robots)
                
        elif formation_type == 'circle':
            # 圆形编队
            radius = 1.0
            return [
                (radius * np.cos(2*np.pi*i/num_robots), 
                 radius * np.sin(2*np.pi*i/num_robots),
                 2*np.pi*i/num_robots + np.pi/2)
                for i in range(num_robots)
            ]
            
        else:
            return self.calculate_formation_offsets('line', num_robots)