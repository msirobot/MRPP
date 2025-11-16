#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import Twist, PoseStamped, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import yaml
import threading
from math import sqrt, atan2
import os

class MultiRobotSim:
    def __init__(self):
        rospy.init_node('multi_robot_simulator')
        
        # 加载配置
        self.load_config()
        
        # 机器人状态
        self.robot_poses = {}
        self.robot_velocities = {}
        self.formation_poses = {}
        
        # 订阅和发布器
        self.cmd_pubs = {}
        self.odom_pubs = {}
        self.pose_subs = {}
        
        # 初始化机器人
        self.setup_robots()
        
        # 导入其他模块
        from multi_robot_sim.motion_planner import MotionPlanner
        from multi_robot_sim.formation_controller import FormationController
        from multi_robot_sim.waypoint_manager import WaypointManager
        
        # 运动规划器
        self.motion_planner = MotionPlanner(self.robot_poses)
        
        # 编队控制器
        self.formation_controller = FormationController(self.robot_poses)
        
        # 航点管理器
        self.waypoint_manager = WaypointManager()
        
        # 线程锁
        self.lock = threading.Lock()
        
        rospy.loginfo("Multi-robot simulation initialized with %d robots", len(self.robot_names))

    def load_config(self):
        """加载机器人配置"""
        config_path = rospy.get_param('~config_path', 
                                    os.path.join(os.path.dirname(__file__), 
                                               '../../config/robots.yaml'))
        try:
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
                self.robot_names = config['robots']
                self.formation_type = config.get('formation_type', 'line')
        except Exception as e:
            rospy.logerr("Failed to load config: %s", e)
            self.robot_names = ['robot1', 'robot2', 'robot3']
            self.formation_type = 'line'

    def setup_robots(self):
        """设置每个机器人的发布器和订阅器"""
        for robot_name in self.robot_names:
            # 命令发布器
            cmd_topic = f"/{robot_name}/cmd_vel"
            self.cmd_pubs[robot_name] = rospy.Publisher(cmd_topic, Twist, queue_size=10)
            
            # 里程计发布器
            odom_topic = f"/{robot_name}/odom"
            self.odom_pubs[robot_name] = rospy.Publisher(odom_topic, Odometry, queue_size=10)
            
            # 位置订阅器 (从Gazebo或实际传感器)
            pose_topic = f"/gazebo/model_states"
            if robot_name not in self.pose_subs:
                self.pose_subs[robot_name] = rospy.Subscriber(
                    pose_topic, ModelStates, 
                    self.model_states_callback
                )
            
            # 初始化状态
            self.robot_poses[robot_name] = [0.0, 0.0, 0.0]  # x, y, theta
            self.robot_velocities[robot_name] = [0.0, 0.0]  # v, w

    def model_states_callback(self, msg):
        """处理Gazebo模型状态更新"""
        with self.lock:
            for i, name in enumerate(msg.name):
                if name in self.robot_names:
                    pose = msg.pose[i]
                    self.robot_poses[name][0] = pose.position.x
                    self.robot_poses[name][1] = pose.position.y
                    
                    # 计算偏航角
                    orientation = pose.orientation
                    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
                    euler = tf.transformations.euler_from_quaternion(quaternion)
                    self.robot_poses[name][2] = euler[2]  # yaw

    def publish_odometry(self, robot_name):
        """发布里程计信息"""
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = f"{robot_name}/base_link"
        
        x, y, theta = self.robot_poses[robot_name]
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        
        quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]
        
        self.odom_pubs[robot_name].publish(odom)

    def send_velocity_command(self, robot_name, linear_x, angular_z):
        """发送速度命令到机器人"""
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pubs[robot_name].publish(cmd)

    def run(self):
        """主运行循环"""
        rate = rospy.Rate(10)  # 10Hz
        
        while not rospy.is_shutdown():
            with self.lock:
                # 动态读取formation_type参数
                self.formation_type = rospy.get_param('~formation_type', self.formation_type)
                
                # 更新运动规划
                target_poses = self.motion_planner.update()
                
                # 更新编队控制
                velocity_commands = self.formation_controller.update(
                    target_poses, self.formation_type
                )
                
                # 发送控制命令
                for robot_name, (v, w) in velocity_commands.items():
                    self.send_velocity_command(robot_name, v, w)
                    
                # 发布里程计
                for robot_name in self.robot_names:
                    self.publish_odometry(robot_name)
            
            rate.sleep()

if __name__ == "__main__":
    try:
        sim = MultiRobotSim()
        sim.run()
    except rospy.ROSInterruptException:
        pass