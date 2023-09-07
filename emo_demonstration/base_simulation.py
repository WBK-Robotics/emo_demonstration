import os
import time
import numpy as np
import pybullet as p
import pybullet_industrial as pi
from emo_demonstration.depth_camera import RGBDCamera


def setup_base_sim(mode = "gui", tool_mode = "camera"):
    file_directory = os.path.dirname(os.path.abspath(__file__))
    sdmbot_urdf_file = os.path.join(file_directory, 'urdf', 'sdmbot copy.urdf')
    main_endeffector_urdf_file = os.path.join(file_directory, 'urdf', 'endeffectorV3.urdf')
    screw_drifer_addon_urdf_file = os.path.join(file_directory, 'urdf', 'screw_driver_addon.urdf')
    camera_addon_urdf_file = os.path.join(file_directory, 'urdf', 'depth_camera.urdf')
    milling_addon_urdf_file = os.path.join(file_directory, 'urdf', 'milling_addon.urdf')

    if mode == "gui":
        physics_client = p.connect(p.GUI, options='--background_color_red=0.5 ' +
                                                '--background_color_green=0.5 ' +
                                                '--background_color_blue=0.5')
                                                
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.resetDebugVisualizerCamera(cameraDistance=0.8, cameraYaw=-90, cameraPitch=-25, cameraTargetPosition=[0,0,0.5])
    elif mode == "direct":
        physics_client = p.connect(p.DIRECT)
    elif mode == "shared_memory":
        physics_client = p.connect(p.SHARED_MEMORY)
    else:
        raise ValueError("Inapropriate value for base sim mode. Available modes are: gui, direct, shared_memory")
    
    p.resetSimulation()
    p.setPhysicsEngineParameter(numSolverIterations=2000)
    p.setVRCameraState(rootPosition=(0,0,-0.50))
    p.setGravity(0,0,-10)


    start_orientation = p.getQuaternionFromEuler([0, 0, 0])

  

    robot = pi.RobotBase(sdmbot_urdf_file, [0, 0, 0], start_orientation)
    gripper = pi.Gripper(main_endeffector_urdf_file, [0, 0, 0], start_orientation, tcp_frame="gripper_center_link")
    gripper.couple(robot,endeffector_name='tool0')

    screwdriver = pi.SuctionGripper(screw_drifer_addon_urdf_file, [0, 0, 0], start_orientation, tcp_frame="screw_tip")
    screwdriver.couple(robot,endeffector_name='tool0')


    if tool_mode == "milling":
        milling_tool = pi.EndeffectorTool(milling_addon_urdf_file, [0, 0, 0], start_orientation, tcp_frame="mill_tip")
        milling_tool.couple(robot, endeffector_name='tool0')
    else:
        camera_parameters = {'width': int(1920/8), 'height': int(1080/8), 'fov': 60,
                             'aspect ratio': 16/9, 'near plane distance': 0.01, 'far plane distance': 10}
        camera = RGBDCamera(camera_addon_urdf_file, [0, 0, 0], start_orientation,
                            camera_parameters=camera_parameters, camera_frame="camera")
        camera.couple(robot,endeffector_name='tool0')

    joint_state = {'shoulder_lift_joint': -np.pi/2,
                   'elbow_joint': -np.pi/2,
                   'wrist_1_joint': -np.pi,
                   'wrist_2_joint': 0,
                   'wrist_3_joint': 0,
                   'shoulder_pan_joint': np.pi/2}
    for _ in range(200):
        robot.set_joint_position(joint_state)
        p.stepSimulation()

    if tool_mode == "milling":
        return robot, gripper, screwdriver, milling_tool
    else:
        return robot, gripper, screwdriver, camera
if __name__ == "__main__":
    import numpy as np

    robot, gripper, screwdriver, camera = setup_base_sim()
    for _ in range(100):
        p.stepSimulation()



    position = np.array([0, 0, 0.63])
    while True:
        p.stepSimulation()
        gripper.actuate(1)
        time.sleep(1)
        for _ in range(50):
            p.stepSimulation()
        gripper.actuate(0)
        time.sleep(1)
        for _ in range(50):
            p.stepSimulation()
