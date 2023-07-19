import os
import time
import pybullet as p
import pybullet_industrial as pi



def setup_base_sim(mode = "gui",returnWhiteboardID = False):
    file_directory = os.path.dirname(os.path.abspath(__file__))
    sdmbot_urdf_file = os.path.join(file_directory, 'urdf', 'sdmbot.urdf')
    endeffector_file = os.path.join(
        file_directory, 'urdf', 'welding_torch.urdf')

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
    # spawn a a rectangular multi body
    rectangle_id = p.createVisualShape(
        p.GEOM_BOX, halfExtents=[0.15, 0.1, 0.01])
    collision_id = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[0.15, 0.1, 0.01])
    whiteboardID = p.createMultiBody(
        baseMass=0, baseVisualShapeIndex=rectangle_id,
        baseCollisionShapeIndex=collision_id,
        basePosition=[0, 0.15,  0.55])

    start_orientation = p.getQuaternionFromEuler([0, 0, 0])

    extruder_properties = {'maximum distance': 0.1,
                           'opening angle': 0,
                           'material': CustomMaterial,
                           'material properties': {'particle size': 0.005, 'color': [0.0, 0.0, 1, 1]},
                           'number of rays': 1}

    robot = pi.RobotBase(sdmbot_urdf_file, [0, 0, 0], start_orientation)
    endeffector = pi.Extruder(
        endeffector_file, [0, 0, 0], start_orientation, extruder_properties)
    endeffector.couple(robot)

    joint_state = {'shoulder_lift_joint': -1.3061111730388184,
                   'elbow_joint': -0.43253040313720703,
                   'wrist_1_joint': 3.6545120912739257,
                   'wrist_2_joint': -1.5724981466876429,
                   'wrist_3_joint': -1.0698006788836878,
                   'shoulder_pan_joint': 1.3991775512695312}

    robot.set_joint_position(joint_state)

    if(returnWhiteboardID):
        return robot, endeffector, whiteboardID
    return robot, endeffector

if __name__ == "__main__":
    import numpy as np

    robot, endeffector = setup_base_sim()

    position = np.array([0, 0, 0.63])
    while True:
        control_loop(endeffector, position)
        p.stepSimulation()
        time.sleep(0.01)
