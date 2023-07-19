import os
import time
import pybullet as p
import pybullet_industrial as pi



def setup_base_sim(mode = "gui"):
    file_directory = os.path.dirname(os.path.abspath(__file__))
    sdmbot_urdf_file = os.path.join(file_directory, 'urdf', 'sdmbot.urdf')
    main_endeffector_urdf_file = os.path.join(file_directory, 'urdf', 'endeffector.urdf')
    screw_drifer_addon_urdf_file = os.path.join(file_directory, 'urdf', 'screw_driver_addon.urdf')

    if mode == "gui":
        physics_client = p.connect(p.GUI, options='--background_color_red=1 ' +
                                                '--background_color_green=1 ' +
                                                '--background_color_blue=1')
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    elif mode == "direct":
        physics_client = p.connect(p.DIRECT)
    elif mode == "shared_memory":
        physics_client = p.connect(p.SHARED_MEMORY)
    else:
        raise ValueError("Inapropriate value for base sim mode. Available modes are: gui, direct, shared_memory")
    
    p.resetSimulation()
    p.setPhysicsEngineParameter(numSolverIterations=2000)
    p.setVRCameraState(rootPosition=(0,0,-0.50))


    start_orientation = p.getQuaternionFromEuler([0, 0, 0])

  

    robot = pi.RobotBase(sdmbot_urdf_file, [0, 0, 0], start_orientation)
    gripper = pi.Gripper(main_endeffector_urdf_file, [0, 0, 0], start_orientation)
    gripper.couple(robot,endeffector_name='tool0')

    screwdriver = pi.SuctionGripper(screw_drifer_addon_urdf_file, [0, 0, 0], start_orientation)
    screwdriver.couple(robot,endeffector_name='tool0')


    joint_state = {'shoulder_lift_joint': -1.3061111730388184,
                   'elbow_joint': -0.43253040313720703,
                   'wrist_1_joint': 3.6545120912739257,
                   'wrist_2_joint': -1.5724981466876429,
                   'wrist_3_joint': -1.0698006788836878,
                   'shoulder_pan_joint': 1.3991775512695312}

    robot.set_joint_position(joint_state)


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
