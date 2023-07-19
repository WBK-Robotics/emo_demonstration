import pybullet as p
import pybullet_industrial as pi
import time
import os 

p.connect(p.GUI)


p.setPhysicsEngineParameter(numSolverIterations=2000)
file_directory = os.path.dirname(os.path.abspath(__file__))
urdf_file = os.path.join(file_directory,  'endeffector.urdf')

robot=pi.RobotBase(urdf_file, [0, 0, 0], [0, 0, 0, 1])

pi.draw_robot_frames(robot)

while True:
    p.stepSimulation()
    time.sleep(1./240.)