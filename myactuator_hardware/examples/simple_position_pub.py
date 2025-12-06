#!/usr/bin/env python3
"""
Simple Joint Position Publisher Example

Directly publishes to joint_trajectory_controller/joint_trajectory topic
for quick testing without using the action interface.

Usage:
    ros2 run myactuator_hardware simple_position_pub.py
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math


class SimplePositionPublisher(Node):
    """Simple publisher that sends position commands."""

    def __init__(self):
        super().__init__('simple_position_publisher')
        
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        
        # Publisher to trajectory topic
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # Timer for periodic sine wave updates
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz
        self.start_time = self.get_clock().now()
        
        # Parameters
        self.amplitude = 1.0  # radians (~57 degrees)
        self.frequency = 0.3  # Hz
        
        self.get_logger().info('Simple Position Publisher started')
        self.get_logger().info('Publishing sine wave to all 4 motors...')
        self.get_logger().info('Press Ctrl+C to stop')

    def timer_callback(self):
        """Publish trajectory point at each timer tick."""
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
        
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        
        # Calculate sine wave positions with phase offsets
        positions = []
        velocities = []
        
        for i in range(4):
            phase = i * (math.pi / 2)  # 0, 90, 180, 270 degrees
            pos = self.amplitude * math.sin(2 * math.pi * self.frequency * elapsed + phase)
            vel = self.amplitude * 2 * math.pi * self.frequency * \
                  math.cos(2 * math.pi * self.frequency * elapsed + phase)
            positions.append(pos)
            velocities.append(vel)
        
        point.positions = positions
        point.velocities = velocities
        point.time_from_start = Duration(sec=0, nanosec=100000000)  # 100ms
        
        trajectory.points.append(point)
        
        self.publisher.publish(trajectory)
        
        # Log periodically
        if int(elapsed * 2) % 2 == 0:
            self.get_logger().info(
                f't={elapsed:.1f}s pos=[{positions[0]:.2f}, {positions[1]:.2f}, '
                f'{positions[2]:.2f}, {positions[3]:.2f}]',
                throttle_duration_sec=0.5
            )


def main():
    rclpy.init()
    node = SimplePositionPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
