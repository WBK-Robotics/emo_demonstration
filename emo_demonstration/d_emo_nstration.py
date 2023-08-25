import os
import time
import pybullet as p
import pybullet_industrial as pi
import numpy as np
import rclpy
import copy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node

from emo_demonstration.base_simulation import setup_base_sim
from emo_demonstration.ur5_trajectory_publisher import URTrajectoryPublisher
from emo_demonstration.io_service_client import IOServiceClient

def load_assembly(folderPath, spawnPoint, spawnOrient=[0, 0, 0], scaleFactor=1.0):
    """Loads all URDFs in the speciied Folder.

    Args:
        folderPath: Path of URDFs to load
        spawnPoint: Point where all URDFS are spawned
        spawnOrient: Orientation used to Spawn objects
        scaleFactor: Scaling Factor applied
    """

    print("loading assembly", folderPath)
    assembly = []
    cid = []
    for subdir, dirs, files in os.walk(folderPath):
        print(subdir, dirs, files)
        files_sorted = sorted(files)
        print(files_sorted)
        fixed = True
        for filename in files_sorted:
            filepath = subdir + os.sep + filename
            if filepath.endswith(".urdf"):
                assembly.append(
                    p.loadURDF(filepath, spawnPoint, spawnOrient, useFixedBase=fixed, globalScaling=scaleFactor))
                fixed=False
                print(filepath)
                print(assembly)
    for i in range(len(assembly)):
        for j in range(i + 1, len(assembly)):
            print(i, j)
            p.setCollisionFilterPair(assembly[i], assembly[j], 0, 0, False)

    #cid.append(p.createConstraint(assembly[0], -1, -1, -1, p.JOINT_FIXED, [0, 0, 1], [0, 0, 0], spawnPoint))
    for i in range(1, len(assembly)):
        cid.append(p.createConstraint(assembly[0], -1, assembly[i], -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0],
                                      [0, 0, 0], [0, 0, 0]))

    return assembly, cid

def move_linear(endeffector: pi.EndeffectorTool, endPoint, delta=0.02):
    """Moving a designated endeffector to the designated endPoint.
    Args:
        endeffector (endeffector_tool): Endeffector to be moved.
        endPoint: Point to move to.
        delta: stepsize of interpolation
        stop: wether to stop at endpoint

    """
    [startPoint, start_orient] = endeffector.get_tool_pose()
    steps = round(np.linalg.norm(np.array(endPoint) - np.array(startPoint)) / delta)
    path = pi.interpolation.linear_interpolation(startPoint, endPoint, steps)
    for pnt in path:
        for _ in range(20):
            endeffector.set_tool_pose(pnt[0], start_orient)
            p.stepSimulation()
        time.sleep(0.01)
    for _ in range(20):
        endeffector.set_tool_pose(endPoint, start_orient)
        p.stepSimulation()       
    #print(f"point {pnt[0]}")
    #move_along_path(endeffector, path, stop)

def get_min_dist(body1, body2, max_distance=1.0):
    distances = []
    for _id1 in range(p.getNumJoints(body1)):
        for _id2 in range(p.getNumJoints(body2)):
            pnts = p.getClosestPoints(bodyA=body1, bodyB=body2, distance=max_distance, linkIndexA=_id1, linkIndexB=_id2)
            if len(pnts) == 0:
                distances.append(max_distance)
            else:
                distances.append(np.min([pt[8] for pt in pnts]))
    return np.min(distances)

def unscrew(screw_Pid, screwdriver, droppoint, constraint=None, screw_depth=-0.00):
    us_pos_up = np.array(p.getLinkState(screw_Pid, 0)[0])
    us_pos_up[2] = us_pos_up[2] + 0.1
    print(f"us_pos_up: {us_pos_up}")
    move_linear(screwdriver, us_pos_up, 0.05)
    for _ in range(200):
        p.stepSimulation()   

    us_target_pos = copy.deepcopy(us_pos_up)
    print(get_min_dist(screwdriver.urdf, screw_Pid) - screw_depth)
    us_target_pos[2] = us_target_pos[2] - get_min_dist(screwdriver.urdf, screw_Pid) - screw_depth
    print(us_target_pos)
    move_linear(screwdriver, us_target_pos, 0.02)
    for _ in range(200):
        p.stepSimulation()
    print(screwdriver.activate())

    if constraint:
        const_save = p.getConstraintInfo(constraint)
        p.removeConstraint(constraint)

    move_linear(screwdriver, us_pos_up, 0.03)
    for _ in range(100):
        p.stepSimulation()  
    print("grippoint2")
    """pos_parent = np.array(p.getLinkState(const_save[0], 0)[0])
    pos_child = np.array(p.getLinkState(const_save[2], 0)[0])
    constraint = p.createConstraint(const_save[0], const_save[1], const_save[2], const_save[3], 
                                    const_save[4], const_save[5], pos_parent, 
                                    pos_parent-pos_child)"""
    pos, orn = p.getBasePositionAndOrientation(screw_Pid)
    p.createConstraint(screw_Pid, -1, -1, -1, p.JOINT_FIXED, [0, 0, 1], [0,0,0],
                       pos,
                       childFrameOrientation= orn)
    screwdriver.deactivate()
    move_linear(screwdriver, droppoint, 0.03)

class SynchronizedSim(Node):

    def __init__(self,robot):
        super().__init__('synchronized_sim')
        self.robot = robot
        self.subscriber = self.create_subscription(JointState,"joint_states",self.custom_callback,10)
        self.subscriber = self.create_subscription(WrenchStamped, "force_torque_sensor_broadcaster/wrench", self.custom_force_callback, 10)
        self.force = np.zeros(3)
        self.joint_position_dict = {}

        
    def custom_callback(self,msg):
        joint_position_dict = {}
        for i in range(len(msg.name)):
            self.joint_position_dict[msg.name[i]] = msg.position[i]

    def custom_force_callback(self, msg):
        self.force[0] = msg.wrench.force.x
        self.force[1] = msg.wrench.force.y
        self.force[2] = msg.wrench.force.z

class emo_controler():
    def __init__(self, robot, synchronization=False):         
        self.robot = robot
        rclpy.init()        
        self.subscriber = SynchronizedSim(self.robot)
        self.trajectory_publisher = URTrajectoryPublisher(robot=self.robot)
        self.gripper_node = IOServiceClient()
        self.synchronization = synchronization     


    def synchronize_sim_pose(self):
        for _ in range(50):
            rclpy.spin_once(self.subscriber)
            self.robot.set_joint_position(self.subscriber.joint_position_dict)
            p.stepSimulation()
    def synchronize_real_pose(self, time_step=2.0, block = True):
        if self.synchronization is False:
            time.sleep(time_step)
            return
        points = [self.trajectory_publisher.get_point()]
        times = [time_step]
        self.trajectory_publisher.set_trajectory(points, times)
        #rclpy.spin_once(self.subscriber)
        if block:
            time.sleep(time_step)
    
    def move_linear(self, endeffector: pi.EndeffectorTool, endPoint, delta=0.1, speed=20.0):
        """Moving a designated endeffector to the designated endPoint.
        Args:
            endeffector (endeffector_tool): Endeffector to be moved.
            endPoint: Point to move to.
            delta: stepsize of interpolation
            speed: aproximated in [mm/s]

        """
        [startPoint, start_orient] = endeffector.get_tool_pose()
        steps = round(np.linalg.norm(np.array(endPoint) - np.array(startPoint)) / delta)
        time_val=delta/speed*1000.0
        path = pi.interpolation.linear_interpolation(startPoint, endPoint, steps+1)
        for pnt in path:
            for _ in range(50):
                endeffector.set_tool_pose(pnt[0], start_orient)
                p.stepSimulation()
            if self.synchronization:
                self.synchronize_real_pose(time_val)
        act_pos, _ = endeffector.get_tool_pose()
        time_val = np.linalg.norm(np.array(endPoint) - np.array(act_pos))/speed*1000
        for _ in range(50):
            endeffector.set_tool_pose(endPoint, start_orient)
            p.stepSimulation()       
        if self.synchronization:
            self.synchronize_real_pose(time_val)      
    
    def force_control_screw(self, endeffector: pi.SuctionGripper, screw_Pid, constraint=None,
                            force_on=2.0, hysteresis=0.2, time_out=5.0, offset=0.00002):
        start_time = time.time()
        pos, orn = endeffector.get_tool_pose()
        pos = list(pos)
        upper_limit = (1.0+hysteresis)*force_on
        lower_limit = (1.0-hysteresis)*force_on
        prev_error = 0
        #active = False
        #first = True
        endeffector.activate()
        for _ in range(5):
            p.stepSimulation()        
        if constraint is not None:
            p.removeConstraint(constraint)

        first = False
        while time.time()-start_time<time_out:
            rclpy.spin_once(self.subscriber)
            #print(f"{self.subscriber.force[1]} {upper_limit} {lower_limit}")
            force_error = self.subscriber.force[1] - force_on
            #if (force_error != prev_error) or self.subscriber.force[1]<0:
            if self.subscriber.force[1]> upper_limit:
                correction = offset + 0.00002*(self.subscriber.force[1]-upper_limit)
            elif self.subscriber.force[1]> force_on:
                correction = offset
            elif self.subscriber.force[1]> lower_limit:
                correction = offset + 0.00001*(self.subscriber.force[1]-force_on)
            else:
                correction = 0.00001*(self.subscriber.force[1]-force_on)
            #correction = -0.00001*force_error
            print(f"force: {self.subscriber.force[1]} error: {force_error} correction: {correction}")

            pos[2]+=correction
            prev_error = force_error            
            endeffector.set_tool_pose(pos, orn)
            for _ in range(5):
                p.stepSimulation()
            self.synchronize_real_pose(0, False)
            #rclpy.spin_once(self.subscriber)
            if correction<0:
                time.sleep(0.01)
        if constraint is not None:
            pos_screw, orn_screw = p.getBasePositionAndOrientation(screw_Pid)
            constraint = p.createConstraint(screw_Pid, -1, -1, -1, p.JOINT_FIXED, [0, 0, 1], [0,0,0],
                                            pos_screw, childFrameOrientation= orn_screw)
        endeffector.deactivate()


        while self.subscriber.force[1]>5:
            rclpy.spin_once(self.subscriber)
            print(f"force: {self.subscriber.force[0]} {self.subscriber.force[1]} {self.subscriber.force[2]} error: {force_error} correction: {correction}")
            pos[2]+=0.001
            endeffector.set_tool_pose(pos, orn)
            for _ in range(10):
                p.stepSimulation()
            self.synchronize_real_pose(0, False)
        return constraint

    def calibrate_with_screw(self, screw_Pid, screwdriver, offset=0.005, wait_time=5):
        us_pos_up = np.array(p.getLinkState(screw_Pid, 0)[0])
        us_pos_up[2] = us_pos_up[2] + 0.1
        print(f"us_pos_up: {us_pos_up}")
        self.move_linear(screwdriver, us_pos_up, 0.05)
        #move_linear(screwdriver, us_pos_up, 0.05)
        us_target_pos = copy.deepcopy(us_pos_up)
        us_target_pos[2] = us_target_pos[2] - get_min_dist(screwdriver.urdf, screw_Pid) + offset
        print(us_target_pos)
        self.move_linear(screwdriver, us_target_pos, 0.02, 10)
        #move_linear(screwdriver, us_target_pos, 0.02)
        time.sleep(wait_time)
        pos2, orn_q = screwdriver.get_tool_pose()
        orn = list(p.getEulerFromQuaternion(orn_q))
        for _ in range(2):
            print(f"orn: {orn}")
            orn[2] += np.pi/180*45
            for _ in range(100):
                screwdriver.set_tool_pose(pos2, p.getQuaternionFromEuler(orn))
                p.stepSimulation()
            self.synchronize_real_pose(2)
        time.sleep(wait_time)
        for _ in range(2):
            print(f"orn: {orn}")
            orn[2] -= np.pi/180*45
            for _ in range(100):
                screwdriver.set_tool_pose(pos2, p.getQuaternionFromEuler(orn))
                p.stepSimulation()
            self.synchronize_real_pose(2)            
        #self.move_linear(screwdriver, us_pos_up, 0.05, 10)
        #print(f"distance to screw: {get_min_dist(screwdriver.urdf, screw_Pid)}")
        #move_linear(screwdriver, us_pos_up, 0.05)
        """while True:
            print(f"distance to screw1: {get_min_dist(screwdriver.urdf, screw_Pid)}")
            print(f"distance to screw2: {get_min_dist(screwdriver.urdf, screw_Pid+1)}")
            print(f"distance to screw3: {get_min_dist(screwdriver.urdf, screw_Pid+2)}")
            print(f"distance to screw4: {get_min_dist(screwdriver.urdf, screw_Pid+3)}")
            print("___________________________________________________")
            self.synchronize_sim_pose()"""
        
    def unscrew(self, screwdriver: pi.SuctionGripper, screw_Pid, constraint=None, offset=0.005):
        us_pos_up = np.array(p.getLinkState(screw_Pid, 0)[0])
        us_pos_up[2] = us_pos_up[2] + 0.08
        print(f"us_pos_up: {us_pos_up}")
        self.move_linear(screwdriver, us_pos_up, 0.1, 40)
        us_target_pos = copy.deepcopy(us_pos_up)
        us_target_pos[2] = us_target_pos[2] - get_min_dist(screwdriver.urdf, screw_Pid) + offset
        print(us_target_pos)
        self.move_linear(screwdriver, us_target_pos, 0.02, 10)
        if self.synchronization:
            constraint = self.force_control_screw(screwdriver, screw_Pid, constraint, 37.0, 0.25, 13, offset=0.00045)
        self.move_linear(screwdriver, us_pos_up, 0.05, 20)
        return constraint
    
    def grip(self, gripper: pi.Gripper, object_Pid, droping_move, constraint = None, offset=[0,0,0], delta_up=0.05, inverted = False):
        us_pos_up = np.array(p.getLinkState(object_Pid, 0)[0]) + np.array(offset)
        us_pos_up[2] = us_pos_up[2] + delta_up
        us_pos_up2 = np.array(p.getLinkState(object_Pid, 0)[0]) + np.array(offset)
        us_pos_up2[2] = us_pos_up2[2] + delta_up
        print(f"us_pos_up: {us_pos_up}")
        self.move_linear(gripper, us_pos_up, 0.1, 40)

        if inverted:
            gripper.actuate(0.0)
            self.gripper_node.close_gripper()
        else:
            gripper.actuate(1.0)
            self.gripper_node.open_gripper()
        for _ in range(50):
            p.stepSimulation()     

        us_target_pos = copy.deepcopy(us_pos_up)
        us_target_pos[2] = us_target_pos[2] - get_min_dist(gripper.urdf, object_Pid)+offset[2]
        
        self.move_linear(gripper, us_target_pos, 0.02, 20)

        pos_g, orn_g = gripper.get_tool_pose()
        inv_pos, inv_orn = p.invertTransform(pos_g, orn_g)
        pos_o, orn_o = p.getBasePositionAndOrientation(object_Pid)
        pos_rest, orn_rest = p.multiplyTransforms(inv_pos, inv_orn, pos_o, orn_o)
        grip_constraint = p.createConstraint(gripper.urdf, gripper._tcp_id,
                                             object_Pid, -1,
                                             p.JOINT_FIXED, [0,0,0],
                                             parentFramePosition=pos_rest,
                                             childFramePosition=[0,0,0],
                                             parentFrameOrientation=orn_rest,
                                             childFrameOrientation=None)
        for _ in range(5):
            p.stepSimulation()

        if inverted:
            gripper.actuate(1.0)
            self.gripper_node.open_gripper()
        else:
            gripper.actuate(0.2)
            self.gripper_node.close_gripper()
        for _ in range(50):
            p.stepSimulation()   

        if constraint is not None:
            p.removeConstraint(constraint)
        for _ in range(50):
            p.stepSimulation()
        self.move_linear(gripper, us_pos_up2, 0.01, 15) 
        droping_move_up = copy.deepcopy(droping_move)    
        droping_move_up[2] = us_pos_up2[2]
        self.move_linear(gripper, droping_move_up, 0.1, 40)
        self.move_linear(gripper, droping_move, 0.1, 40)
        p.removeConstraint(grip_constraint)  
        if inverted:
            gripper.actuate(0.0)
            self.gripper_node.close_gripper()
        else:
            gripper.actuate(1.0)
            self.gripper_node.open_gripper()
        for _ in range(50):
            p.stepSimulation()  
        self.move_linear(gripper, droping_move_up, 0.1, 40)
        return pos_g, orn_g
     
    
    def switch_to_gripper(self):
        joint_state = { 'shoulder_lift_joint': -np.pi/2,
                        'elbow_joint': -np.pi/2,
                        'wrist_1_joint': -np.pi,
                        'wrist_2_joint': 0,
                        'wrist_3_joint': 0,
                        'shoulder_pan_joint': np.pi/2}
        for _ in range(50):
            self.robot.set_joint_position(joint_state)
            p.stepSimulation()
        self.synchronize_real_pose(2)
        joint_state["wrist_1_joint"]=-np.pi/2
        """for _ in range(50):
            self.robot.set_joint_position(joint_state)
            p.stepSimulation()"""
        self.synchronize_real_pose(2)        
        joint_state["wrist_2_joint"]=np.pi/2
        for _ in range(50):
            self.robot.set_joint_position(joint_state)
            p.stepSimulation()
        self.synchronize_real_pose(2)   

    def switch_to_screwdriver(self):
        joint_state = { 'shoulder_lift_joint': -np.pi/2,
                        'elbow_joint': -np.pi/2,
                        'wrist_1_joint': -np.pi/2,
                        'wrist_2_joint': np.pi/2,
                        'wrist_3_joint': 0,
                        'shoulder_pan_joint': np.pi/2}
        for _ in range(50):
            self.robot.set_joint_position(joint_state)
            p.stepSimulation()
        self.synchronize_real_pose(2)
        joint_state["wrist_2_joint"]=0
        """for _ in range(50):
            self.robot.set_joint_position(joint_state)
            p.stepSimulation()"""
        self.synchronize_real_pose(2)        
        joint_state["wrist_1_joint"]=-np.pi
        for _ in range(50):
            self.robot.set_joint_position(joint_state)
            p.stepSimulation()
        self.synchronize_real_pose(2)   
def main():
    file_directory = os.path.dirname(os.path.abspath(__file__))
    motor_path = os.path.join(file_directory,'urdf', 'DM_EMO_urdf')    
    robot, gripper, screwdriver, camera = setup_base_sim()
    controler = emo_controler(robot, synchronization=True)  
    controler.synchronize_real_pose(3.0)
    controler.gripper_node.open_gripper()
    gripper.actuate(1.0)

  
    #controler.switch_to_gripper()
    #controler.switch_to_screwdriver()
    

    spawn_point = np.array([-0.0015, -0.003, 0.005])
    spawn_orient = p.getQuaternionFromEuler([0, 0, np.pi/180 * 0.0])
    motor, constraint_ids = load_assembly(motor_path, spawn_point, spawn_orient, 0.001)

    ###############################################EMO
    """i=7
    for i in range(3,i):
        constraint_ids[i-1] = controler.unscrew(screwdriver, motor[i], constraint_ids[i-1], -0.002)
    #constraint_ids[i-1] = controler.unscrew(screwdriver, motor[i], constraint_ids[i-1], -0.002)
    controler.switch_to_gripper()
    for i in range(3,i):
        controler.grip(gripper, motor[i], [0.0, -0.3, 0.12], constraint_ids[i-1], [0,-0.00,-0.013],0.150)"""
    #controler.grip(gripper, motor[i], [[0.10, 0.105, 0.8]], constraint_ids[i-1], [0,0.00,-0.01],0.125)
    original_pos = {}
    controler.switch_to_gripper()
    """i=1
    original_pos[i], _ = controler.grip(gripper, motor[i], [-0.2, 0.075, 0.01], constraint_ids[i-1], [0, -0.0565,-0.02], 0.06, False)
    i=2
    original_pos[i], _ = controler.grip(gripper, motor[i], [-0.2, 0.0, 0.101], constraint_ids[i-1], [0, -0.012,-0.025], 0.19, False)"""
    i=7
    original_pos[i], _ = controler.grip(gripper, motor[i] ,[-0.2, -0.150, 0.057], constraint_ids[i-1], [0, -0.042,-0.03], 0.1, False)

    i=7
    original_pos[i], _ = controler.grip(gripper, motor[i] ,original_pos[i], constraint_ids[i-1], [0, -0.042,-0.03], 0.1, False)

    #####################################################EMO
    """initial_position = spawn_point + np.array([-0.0, 0.0, 0.2]) #np.array([0, 0.2, 1.0])
    initial_orientation = p.getQuaternionFromEuler([np.pi/2,0,-np.pi/2])
    for _ in range(100):
        screwdriver.set_tool_pose(initial_position, initial_orientation)
        p.stepSimulation()
        #time.sleep(0.01)
    controler.synchronize_real_pose(2.0)

    controler.calibrate_with_screw(motor[3], screwdriver, 0.002, 5)
    controler.calibrate_with_screw(motor[4], screwdriver, 0.002, 5)
    controler.calibrate_with_screw(motor[5], screwdriver, 0.002, 5)
    controler.calibrate_with_screw(motor[6], screwdriver, 0.002, 5)"""

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


    