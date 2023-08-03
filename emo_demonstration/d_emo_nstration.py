import os
import time
import pybullet as p
import pybullet_industrial as pi
import numpy as np
import rclpy

from emo_demonstration.base_simulation import setup_base_sim
from emo_demonstration.ur5_trajectory_publisher import URTrajectoryPublisher

def main():
    rclpy.init()
    robot, gripper, screwdriver = setup_base_sim()
    trajectory_publisher = URTrajectoryPublisher(robot=robot)

    screwdriver.set_tool_pose([0,0.2,0.7], p.getQuaternionFromEuler([np.pi/2,0,0]))
    for _ in range(100):
        p.stepSimulation()

    points = [trajectory_publisher.get_point(), trajectory_publisher.get_point()]
    times = [2.0,2.1]
    
    while True:
        trajectory_publisher.set_trajectory(points, times)
        time.sleep(3)
    