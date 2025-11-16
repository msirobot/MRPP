# 多小车ROS仿真系统

## 功能特点
- 多机器人协同运动
- 运动规划与航点跟踪
- 多种编队控制（直线、三角形、圆形）
- Gazebo仿真环境
- RVIZ可视化

## 安装依赖
```bash
sudo apt-get install ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control \
ros-noetic-robot-state-publisher ros-noetic-xacro ros-noetic-rviz
```

## 编译
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 使用方法

### 启动多机器人仿真
```bash
roslaunch multi_robot_simulation multi_robot.launch
```

此命令将：
- 启动Gazebo仿真环境
- 生成三个差速驱动机器人（robot1, robot2, robot3）
- 启动多机器人控制节点
- 启动RVIZ可视化

### 查看话题
机器人话题：
```bash
rostopic list | grep robot
# /robot1/cmd_vel  - 机器人1速度控制
# /robot1/odom     - 机器人1里程计信息
# /robot2/cmd_vel  - 机器人2速度控制
# /robot2/odom     - 机器人2里程计信息
# /robot3/cmd_vel  - 机器人3速度控制
# /robot3/odom     - 机器人3里程计信息
```

### 动态切换编队类型
支持的编队类型：`line`（直线）、`triangle`（三角形）、`circle`（圆形）

```bash
# 切换到三角形编队
rosparam set /multi_robot_sim/formation_type triangle

# 切换到圆形编队
rosparam set /multi_robot_sim/formation_type circle

# 切换到直线编队
rosparam set /multi_robot_sim/formation_type line
```

### 配置文件

#### robots.yaml
机器人配置文件，位于 `config/robots.yaml`：
```yaml
robots:
  - robot1
  - robot2
  - robot3
formation_type: "line"  # line, triangle, circle
control_frequency: 10
```

#### waypoints.yaml
航点配置文件，位于 `config/waypoints.yaml`，定义机器人编队的目标航点。

#### formations.yaml
编队配置文件，位于 `config/formations.yaml`，定义各种编队的详细参数。

## 节点说明

### multi_robot_sim
主控制节点，负责：
- 订阅各机器人的里程计信息
- 发布速度控制命令
- 协调编队控制
- 管理航点跟踪

参数：
- `~config_path`: 机器人配置文件路径
- `~formation_type`: 编队类型（line/triangle/circle）

## 故障排除

### 机器人未生成
- 确认Gazebo已正确安装
- 检查URDF/XACRO文件语法
- 查看日志：`rosnode info /multi_robot_sim`

### 机器人不移动
- 确认控制节点正在运行：`rosnode list | grep multi_robot_sim`
- 检查速度命令是否发布：`rostopic echo /robot1/cmd_vel`
- 查看里程计数据：`rostopic echo /robot1/odom`