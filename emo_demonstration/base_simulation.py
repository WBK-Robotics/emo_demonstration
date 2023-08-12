import os
import time
import numpy as np
import pybullet as p
import pybullet_industrial as pi



def setup_base_sim(mode = "gui"):
    file_directory = os.path.dirname(os.path.abspath(__file__))
    sdmbot_urdf_file = os.path.join(file_directory, 'urdf', 'sdmbot.urdf')
    main_endeffector_urdf_file = os.path.join(file_directory, 'urdf', 'endeffector.urdf')
    screw_drifer_addon_urdf_file = os.path.join(file_directory, 'urdf', 'screw_driver_addon.urdf')

    if mode == "gui":
        physics_client = p.connect(p.GUI, options='--background_color_red=0.5 ' +
                                                '--background_color_green=0.5 ' +
                                                '--background_color_blue=0.5')
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=-90, cameraPitch=-25, cameraTargetPosition=[0,0,0.5])
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


    joint_state = {'shoulder_lift_joint': -np.pi/2,
                   'elbow_joint': -np.pi/2,
                   'wrist_1_joint': -np.pi,
                   'wrist_2_joint': 0,
                   'wrist_3_joint': 0,
                   'shoulder_pan_joint': np.pi/2}
    for _ in range(200):
        robot.set_joint_position(joint_state)
        p.stepSimulation()

    return robot, gripper, screwdriver

if __name__ == "__main__":
    import numpy as np

    robot, gripper, screwdriver = setup_base_sim()
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
