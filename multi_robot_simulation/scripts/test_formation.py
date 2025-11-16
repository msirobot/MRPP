#!/usr/bin/env python3

import rospy
import subprocess
import time

def test_formation():
    """测试不同的编队形式"""
    
    # 启动仿真
    print("Starting simulation...")
    launch_process = subprocess.Popen([
        'roslaunch', 'multi_robot_simulation', 'multi_robot.launch'
    ])
    
    time.sleep(10)  # 等待系统启动
    
    # 测试不同编队
    formations = ['line', 'triangle', 'circle']
    
    for formation in formations:
        print(f"Testing {formation} formation...")
        
        # 设置编队参数
        subprocess.call([
            'rosparam', 'set', '/multi_robot_sim/formation_type', formation
        ])
        
        time.sleep(15)  # 运行15秒
    
    # 关闭仿真
    print("Stopping simulation...")
    launch_process.terminate()
    launch_process.wait()

if __name__ == "__main__":
    test_formation()