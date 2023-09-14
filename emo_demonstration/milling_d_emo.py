import os
import time
import pybullet as p
import pybullet_industrial as pi
import numpy as np
import rclpy
import copy
import math
import matplotlib.pyplot as plt
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from PIL import Image

from emo_demonstration.base_simulation import setup_milling_sim
from emo_demonstration.ur5_trajectory_publisher import URTrajectoryPublisher
from emo_demonstration.io_service_client import IOServiceClient
from emo_demonstration.d_emo_nstration import emo_controler, load_assembly

def main():
    file_directory = os.path.dirname(os.path.abspath(__file__))
    
    seq_path = os.path.join(file_directory,'Seq', 'plan')    
    action_path = os.path.join(file_directory,'Seq', 'Actions')    
    robot, gripper, screwdriver, milling_tool = setup_milling_sim()
    
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111)
    mngr = plt.get_current_fig_manager()
    mngr.window.wm_geometry("+0+0")



    controler = emo_controler(robot, synchronization=True)  
    controler.synchronize_real_pose(3.0)
    controler.gripper_node.open_gripper()
    gripper.actuate(1.0)

  
    #controler.switch_to_gripper()
    #controler.switch_to_screwdriver()
    

    spawn_point = np.array([-0.0015, -0.003, 0.005])
    spawn_orient = p.getQuaternionFromEuler([0, 0, np.pi/180 * 0.0])


 
    print("PRESS 1 TO START NONDESTRUCTIVE DISASSEMBLY 2 TO START DESTRUCTIVE DISASSEMBLY")
    mode = "nondestructive"
    waiting = True
    while waiting:
        p.stepSimulation()
        keys = p.getKeyboardEvents()
        for key, state in keys.items():
            if state == p.KEY_IS_DOWN:
                if key == ord('1'):
                    waiting=False
                    mode = "nondestructive"
                elif key == ord('2'):
                    waiting=False
                    mode = "destructive"
    print(mode)
    if mode == "nondestructive":
        print("loading motor...")
        motor_path = os.path.join(file_directory,'urdf', 'DM_EMO_urdf') 
        motor, constraint_ids, part_names = load_assembly(motor_path, spawn_point, spawn_orient, 0.001)
        part_names = [name[:-4] for name in part_names]
        print("done")
        time.sleep(3600)
        """i=part_names.index("S4")+1
        for i in range(3,i):
            controler.switch_to_screwdriver()            
            constraint_ids[i-1] = controler.unscrew(screwdriver, motor[i], constraint_ids[i-1], -0.002)
            print(i)
            controler.switch_to_gripper()
            controler.grip(gripper, motor[i], [0.0, -0.3, 0.12], constraint_ids[i-1], [0,-0.00,-0.013],0.150)"""
        controler.switch_to_gripper()
        i=part_names.index("L2")
        controler.grip(gripper, motor[i], [-0.2, 0.075, 0.01], constraint_ids[i-1], [0, -0.056,-0.02], 0.06, False)
        i=part_names.index("Ro")
        controler.grip(gripper, motor[i], [-0.2, 0.0, 0.101], constraint_ids[i-1], [0, -0.012,-0.025], 0.19, False)
    elif mode == "destructive":
        print("loading motor...")
        motor_path = os.path.join(file_directory,'urdf', 'milled_Motor2') 
        motor, constraint_ids, part_names = load_assembly(motor_path, spawn_point, spawn_orient, 0.001)
        part_names = [name[:-4] for name in part_names]
        print(part_names)
        print("done")       
        controler.switch_to_milling_tool()
        #controler.execute_milling(milling_tool, os.path.join(file_directory,"Seq","tool_path_0_1.txt"), spawn_point, spawn_orient,1.0)  
        print("please deaktivate dremel and press 3")
        waiting = True
        while waiting:
            p.stepSimulation()
            keys = p.getKeyboardEvents()
            for key, state in keys.items():
                if state == p.KEY_IS_DOWN:
                    if key == ord('3'):
                        waiting=False
 
        controler.switch_to_gripper()
        i=part_names.index("L2_milled_Ro_2")
        controler.grip(gripper, motor[i], [-0.2, 0.075, 0.01], constraint_ids[i-1], [0.0275, 0.00, -0.022], 0.19, False)
        i=part_names.index("Ro")
        controler.grip(gripper, motor[i], [-0.2, 0.0, 0.101], constraint_ids[i-1], [-0.001, -0.011,-0.025], 0.19, False)
    
    while True:
        p.stepSimulation()
        time.sleep(0.1)

    

    #controler.calibrate_with_screw(motor[3], screwdriver, 0.0, 5)
    #time.sleep(3600)
    #for i in range(3,7):
    #    controler.calibrate_with_screw(motor[i], screwdriver, offset=0.00, wait_time=3)
    for i in range(3,7):
        constraint_ids[i-1] = controler.unscrew(screwdriver, motor[i], constraint_ids[i-1], -0.002)

    """while True:
        rclpy.spin_once(subscriber)
        p.stepSimulation()
        print(subscriber.force[0])
"""


    