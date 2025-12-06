#!/usr/bin/env python3
"""
Test script to send trajectory commands to the multi-motor robot.

This script demonstrates how to send joint trajectory commands to the 
JointTrajectoryController, which is compatible with MoveIt2.

Usage:
  ros2 run myactuator_hardware test_trajectory.py

Or directly:
  python3 test_trajectory.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math


class TrajectoryTestNode(Node):
    def __init__(self):
        super().__init__('trajectory_test_node')
        
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Action server available!')

    def send_trajectory(self, positions_list, durations_list):
        """
        Send a trajectory with multiple waypoints.
        
        Args:
            positions_list: List of [joint1, joint2, joint3] positions in radians
            durations_list: List of durations from start for each waypoint
        """
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['joint_1', 'joint_2', 'joint_3']
        
        for positions, duration in zip(positions_list, durations_list):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = [0.0, 0.0, 0.0]
            point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1) * 1e9))
            goal.trajectory.points.append(point)
        
        self.get_logger().info(f'Sending trajectory with {len(positions_list)} waypoints...')
        
        send_goal_future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted!')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        positions = feedback.actual.positions
        self.get_logger().info(f'Current positions: [{positions[0]:.3f}, {positions[1]:.3f}, {positions[2]:.3f}]')

    def result_callback(self, future):
        result = future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Trajectory execution successful!')
        else:
            self.get_logger().error(f'Trajectory execution failed with error code: {result.error_code}')


def main():
    rclpy.init()
    node = TrajectoryTestNode()
    
    # Example trajectory: move joints in sequence
    positions = [
        [0.0, 0.0, 0.0],           # Home position
        [math.pi/4, 0.0, 0.0],     # Joint 1 to 45 degrees
        [math.pi/4, math.pi/4, 0.0],  # Joint 2 to 45 degrees
        [math.pi/4, math.pi/4, math.pi/4],  # Joint 3 to 45 degrees
        [0.0, 0.0, 0.0],           # Back to home
    ]
    durations = [1.0, 2.0, 3.0, 4.0, 5.0]
    
    node.send_trajectory(positions, durations)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
