import os
import time
import numpy as np
import pybullet as p
import pybullet_industrial as pi
import time

from base_simulation import setup_milling_sim

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
                # print(filepath)
                # print(assembly)
    for i in range(len(assembly)):
        for j in range(i + 1, len(assembly)):
            print(i, j)
            p.setCollisionFilterPair(assembly[i], assembly[j], 0, 0, False)

    #cid.append(p.createConstraint(assembly[0], -1, -1, -1, p.JOINT_FIXED, [0, 0, 1], [0, 0, 0], spawnPoint))
    for i in range(1, len(assembly)):
        cid.append(p.createConstraint(assembly[0], -1, assembly[i], -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0],
                                      [0, 0, 0], [0, 0, 0]))

    return assembly, cid, files_sorted

def execute_milling(endeffector, spawn_point, spawn_orient, path_file):
    with open(path_file, "r") as file:
        tool_path = []
        for line in file:
            x, y, z = map(float, line.strip().split())
            pnt, _ = p.multiplyTransforms(spawn_point, spawn_orient, np.array([x,y,z])/1000, [0,0,0,1])
            tool_path.append(np.array(pnt))
    [startPoint, start_orient] = endeffector.get_tool_pose()
    endeffector.set_tool_pose(tool_path[0] + np.array([0,0,0.01]), start_orient)
    p.stepSimulation()
    endeffector.set_tool_pose(tool_path[0], start_orient)
    p.stepSimulation()
    for i in range(1, len(tool_path)):
        endeffector.set_tool_pose(tool_path[i], start_orient)
        p.stepSimulation()
        p.addUserDebugLine(tool_path[i], tool_path[i-1],lineColorRGB=[1.0, 0,0], lineWidth=2.0, lifeTime=0)
    endeffector.set_tool_pose(tool_path[-1] + np.array([0,0,0.02]), start_orient)
    p.stepSimulation()

def switch_to_milling_tool(robot):
        joint_state = { 'shoulder_lift_joint': -np.pi/2,
                        'elbow_joint': -np.pi/2,
                        'wrist_1_joint': -np.pi/2,
                        'wrist_2_joint': np.pi/2,
                        'wrist_3_joint': 0,
                        'shoulder_pan_joint': np.pi/2}
        for _ in range(50):
            robot.set_joint_position(joint_state)
            p.stepSimulation()
        #time.sleep(10)
        #self.synchronize_real_pose(2)
        joint_state["wrist_2_joint"]=0
        """for _ in range(50):
            self.robot.set_joint_position(joint_state)
            p.stepSimulation()"""
        #self.synchronize_real_pose(2)
        joint_state["wrist_1_joint"]=-np.pi
        for _ in range(50):
            robot.set_joint_position(joint_state)
            p.stepSimulation()
        #time.sleep(10)
        #self.synchronize_real_pose(2)
        joint_state["wrist_3_joint"]=-np.pi
        for _ in range(50):
            robot.set_joint_position(joint_state)
            p.stepSimulation()
        # self.synchronize_real_pose(2)

if __name__ == "__main__":
    print("start")
    pose = []
    orientation = []
    file_directory = os.path.dirname(os.path.abspath(__file__))
    motor_path = os.path.join(file_directory, 'urdf', 'DM_EMO_urdf')

    robot, gripper, screwdriver, milling_tool = setup_milling_sim()
    for _ in range(100):
        p.stepSimulation()

    spawn_point = np.array([0.025, -0.025, 0])
    spawn_orient = p.getQuaternionFromEuler([0, 0, 0])
    motor, constraint_ids, part_names = load_assembly(motor_path, spawn_point, spawn_orient, 0.001)

    part_names = [name[:-4] for name in part_names]
    dict_assembly = {motor_id: part for (motor_id, part) in zip(motor, part_names)}
    switch_to_milling_tool(robot)

    execute_milling(milling_tool, spawn_point, spawn_orient, os.path.join(file_directory,"Seq", "tool_path_0_1.txt"))

    while True:
        p.stepSimulation()
    print("done")
