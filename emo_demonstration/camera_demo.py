import os
import time
import numpy as np
import pybullet as p
import pybullet_industrial as pi
import matplotlib.pyplot as plt

from base_simulation import setup_base_sim


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

def switch_to_camera(robot):
    joint_state = {'shoulder_lift_joint': -np.pi / 2,
                   'elbow_joint': -np.pi / 2,
                   'wrist_1_joint': -np.pi,
                   'wrist_2_joint': 0,
                   'wrist_3_joint': 0,
                   'shoulder_pan_joint': np.pi / 2}
    for _ in range(50):
        robot.set_joint_position(joint_state)
        p.stepSimulation()
    joint_state["wrist_1_joint"] = -np.pi / 2
    for _ in range(50):
        robot.set_joint_position(joint_state)
        p.stepSimulation()
    joint_state["wrist_2_joint"] = np.pi / 2
    for _ in range(50):
        robot.set_joint_position(joint_state)
        p.stepSimulation()

def calculate_point_cloud(depth_data, view_matrix, proj_matrix):

    height, width = depth_data.shape

    # create a 4x4 transform matrix that goes from pixel coordinates (and depth values) to world coordinates
    proj_matrix = np.asarray(proj_matrix).reshape([4, 4], order="F")
    view_matrix = np.asarray(view_matrix).reshape([4, 4], order="F")
    tran_pix_world = np.linalg.inv(np.matmul(proj_matrix, view_matrix))

    # create a grid with pixel coordinates and depth values
    y, x = np.mgrid[-1:1:2 / height, -1:1:2 / width]
    y *= -1.
    x, y, z = x.reshape(-1), y.reshape(-1), depth_data.reshape(-1)
    h = np.ones_like(z)

    pixels = np.stack([x, y, z, h], axis=1)
    # filter out "infinite" depths
    pixels = pixels[z < 1.0]
    pixels[:, 2] = 2 * pixels[:, 2] - 1

    # turn pixels to world coordinates
    points = np.matmul(tran_pix_world, pixels.T).T
    points /= points[:, 3: 4]
    points = points[:, :3]

    return points

if __name__ == "__main__":

    file_directory = os.path.dirname(os.path.abspath(__file__))
    motor_path = os.path.join(file_directory,'urdf', 'DM_EMO_urdf')

    robot, gripper, screwdriver, camera = setup_base_sim()
    for _ in range(100):
        p.stepSimulation()

    switch_to_camera(robot)

    spawn_point = np.array([-0.0015, -0.003, 0.005])
    spawn_orient = p.getQuaternionFromEuler([0, 0, np.pi/180 * 0.0])
    motor, constraint_ids = load_assembly(motor_path, spawn_point, spawn_orient, 0.001)


    initial_position = spawn_point + np.array([-0.0, 0.0, 0.3])  # np.array([0, 0.2, 1.0])
    initial_orientation = p.getQuaternionFromEuler([-np.pi, 0, 0])
    for _ in range(100):
        camera.set_tool_pose(initial_position, initial_orientation)
        p.stepSimulation()
        time.sleep(0.01)



    # Create a sample array (100x100 with random values)
    data, depth_data, _ = camera.get_image()
    points = calculate_point_cloud(depth_data, camera.get_view_matrix(), camera.projection_matrix)
    # Save using matplotlib
    plt.imsave('D:\wd_stuff\output_image_matplotlib.png', data.astype(np.uint8))

    # Create a new figure
    fig = plt.figure()

    # Add 3d axes
    ax = fig.add_subplot(111, projection='3d')

    # Scatter plot
    ax.scatter(points[:, 0], points[:, 1], points[:, 2])

    # Setting the labels
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    # Show the plot
    plt.show()