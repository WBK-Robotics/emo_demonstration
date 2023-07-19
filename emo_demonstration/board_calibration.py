
import time

import numpy as np
import pybullet as p








def control_loop(endeffector,position,orientation = [0,0,0,1]):
    keys = p.getKeyboardEvents()
    shift = 0.0005

    if p.B3G_LEFT_ARROW in keys:
        position[1] = position[1] - shift
    if p.B3G_RIGHT_ARROW in keys:
        position[1] = position[1] + shift
    if p.B3G_UP_ARROW in keys:
        position[0] = position[0] + shift
    if p.B3G_DOWN_ARROW in keys:
        position[0] = position[0] - shift
    if p.B3G_SPACE in keys:
        position[2] = position[2] - shift
    if p.B3G_DELETE in keys:
        position[2] = position[2] + shift

    endeffector.set_tool_pose(position, orientation)



    for _ in range(3):
        p.stepSimulation()

    return position


def main():
    from emo_demonstration.ur5_continous_publisher import URContPublisher
    from emo_demonstration.base_simulation import setup_base_sim

    import rclpy
    rclpy.init()

    robot, endeffector = setup_base_sim()
    p.setRealTimeSimulation(True)
    time.sleep(0.5)
    publisher_node = URContPublisher(robot)

    position = np.array([0, 0, 0.66])

    
    for _ in range(10):
        control_loop(endeffector, position)
    time.sleep(1)
    print("Transmission started!")
    while(True):

        control_loop(endeffector, position)

        print(position[0],position[1],position[2])
        publisher_node.set_joints()
        p.stepSimulation()
        time.sleep(0.01)


if __name__ == "__main__":
    main()
