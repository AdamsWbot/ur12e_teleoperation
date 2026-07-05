#!/usr/bin/env python3
"""
测试与真实UR12e的连接，读取关节位置和TCP位姿。
使用前请确保机器人运行在远程控制模式，且IP地址正确。
"""

import sys
sys.path.append("src")

import logging
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface

ROBOT_IP = "1.2.3.4"   # 请根据实际修改///1111111需要协调设计修改

def main():
    logging.basicConfig(level=logging.INFO)
    logging.info("Attempting to connect to UR12e at %s...", ROBOT_IP)

    try:
        rtde_r = RTDEReceiveInterface(ROBOT_IP)
        rtde_c = RTDEControlInterface(ROBOT_IP)
        logging.info("Connection successful!")

        # 读取关节位置
        joint_pos = rtde_r.getActualQ()
        print("Joint positions (rad):", joint_pos)

        # 读取工具中心点位姿
        tcp_pose = rtde_r.getActualTCPPose()
        print("TCP pose [x,y,z,rx,ry,rz]:", tcp_pose)

        rtde_c.disconnect()
        rtde_r.disconnect()

    except Exception as e:
        logging.error("Connection failed: %s", e)
        sys.exit(1)

if __name__ == "__main__":
    main()