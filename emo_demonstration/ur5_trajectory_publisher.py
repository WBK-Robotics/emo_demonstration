

from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def load_points_from_csv(filename):
    import csv
    points = []
    with open(filename, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for i,row in enumerate(reader):
            if i >0:
                if any(row):
                    point = JointTrajectoryPoint()
                    point.positions = [float(x) for x in row]
                    points.append(point)
    return points


class URTrajectoryPublisher(Node):

    def __init__(self, robot):
        super().__init__('pub_joint_state')
        self.publisher = self.create_publisher(
            JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        self.robot = robot

        self.joint_names = ['shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint', 'shoulder_pan_joint']

    def get_point(self):
        joint_states = self.robot.get_joint_state()
        point = JointTrajectoryPoint()
        point.positions = [joint_states[key]['position']
                           for key in self.joint_names]

        return point

    def set_trajectory(self, points, timings):
        # set timing of trajectory points:
        for i in range(len(points)):
            seconds, nanoseconds = self.convert_time(timings[i])
            points[i].time_from_start.sec = seconds
            points[i].time_from_start.nanosec = nanoseconds

        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = self.joint_names
        joint_trajectory.points = points
        self.publisher.publish(joint_trajectory)

    @staticmethod
    def convert_time(time_in_seconds):
        seconds = int(time_in_seconds)
        nanoseconds = int((time_in_seconds-seconds)*1e9)
        return seconds, nanoseconds


def main():
    from time import sleep

    import numpy as np
    import pybullet as p
    import rclpy
    from emo_demonstration.base_simulation import setup_base_sim
    rclpy.init()
    robot, endeffector,_ = setup_base_sim(mode="direct")
    publisher_node = URTrajectoryPublisher(robot)

    points = load_points_from_csv('test.csv')
    
    timing= np.arange(len(points))*0.1+2

    while True:
        publisher_node.set_trajectory(points, timing)
        sleep(timing[-1]+2)


if __name__ == "__main__":

    main()
