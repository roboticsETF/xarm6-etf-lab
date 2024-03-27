#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from builtin_interfaces.msg import Duration
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand as GripperCommandAction
from control_msgs.msg import GripperCommand
import numpy as np

class TestMoveRealXarm6Node(Node):

    def __init__(self):
        super().__init__('test_move_real_xarm6_node')
        self.period = 5.0
        self.timer = self.create_timer(self.period, self.timer_callback)
        self.trajectory_publisher = self.create_publisher(JointTrajectory, '/xarm6_traj_controller/joint_trajectory', 10)
        self.gripper_client = ActionClient(self, GripperCommandAction, '/xarm_gripper/gripper_action')

        self.joints = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.gripper_joint = ['drive_joint']
        self.counter = 0
    
    def goHome(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joints
        c = np.pi/180

        joint_point = JointTrajectoryPoint()
        joint_point.positions = [0*c, 0*c, 0*c, 180*c, 90*c, 0*c]
        joint_point.time_from_start = Duration(sec=self.period)
        trajectory.points.append(joint_point)
        self.trajectory_publisher.publish(trajectory)
    
    def moveGripper(position, max_effort = 5.0):
        gripper_command = GripperCommandAction.Goal()
        gripper_command.command = GripperCommand(position = 1.0 - position, max_effort = max_effort)    # Inverse logic
        self.gripper_client.send_goal_async(gripper_command)

    def move1(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joints
        c = np.pi/180

        joint_point = JointTrajectoryPoint()
        joint_point.positions = [0*c, 22*c, -40*c, 180*c, 70*c, 0*c]
        joint_point.time_from_start = Duration(sec=2)
        trajectory.points.append(joint_point)
        self.trajectory_publisher.publish(trajectory)
        
    def move2(self):
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joints
        c = np.pi/180

        joint_point = JointTrajectoryPoint()
        joint_point.positions = [45*c, 22*c, -40*c, 180*c, 70*c, 0*c]       
        joint_point.time_from_start = Duration(sec=2)
        trajectory.points.append(joint_point)

        joint_point = JointTrajectoryPoint()
        joint_point.positions = [45*c, 22*c, -40*c, 180*c, 70*c, -90*c]        
        joint_point.time_from_start = Duration(sec=3)
        trajectory.points.append(joint_point)

        joint_point = JointTrajectoryPoint()
        joint_point.positions = [45*c, 22*c, -40*c, 180*c, 70*c, -90*c]        
        joint_point.time_from_start = Duration(sec=4)
        trajectory.points.append(joint_point)

        joint_point = JointTrajectoryPoint()
        joint_point.positions = [45*c, 22*c, -40*c, 180*c, 70*c, 0*c]        
        joint_point.time_from_start = Duration(sec=5)
        trajectory.points.append(joint_point)

        self.trajectory_publisher.publish(trajectory)


    def timer_callback(self):
        if self.counter == 0:
            print("Going home...")
            self.goHome()
            self.moveGripper(position = 1.0)
            self.counter += 1
        elif self.counter == 1:
            print("Move 1")
            self.move1()
            self.counter += 1
        elif self.counter == 2:
            print("Move 2")
            self.move2()
            self.moveGripper(position = 0.1)
            self.counter += 1
        elif self.counter == 3:
            print("Going home...")
            self.goHome()
            self.moveGripper(position = 1.0)
            self.counter = 1

        print("----------------------------------")
        
        
def main(args=None):
    rclpy.init(args=args)
    joint_publisher = TestMoveRealXarm6Node()
    rclpy.spin(joint_publisher)
    joint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
