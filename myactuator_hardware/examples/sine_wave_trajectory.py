#!/usr/bin/env python3
"""
Sine Wave Trajectory Example for 4 Motors

This script sends sine wave trajectories to 4 motors via JointTrajectoryController,
simulating MoveIt2-style trajectory commands.

Usage:
    ros2 run myactuator_hardware sine_wave_trajectory.py

    # With custom parameters
    ros2 run myactuator_hardware sine_wave_trajectory.py --amplitude 1.0 --frequency 0.5 --duration 10.0
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import argparse


class SineWaveTrajectoryNode(Node):
    """Node that sends sine wave trajectories to joint_trajectory_controller."""

    def __init__(self, amplitude: float = 1.0, frequency: float = 0.3, 
                 duration: float = 10.0, points_per_cycle: int = 50):
        super().__init__('sine_wave_trajectory_node')
        
        self.amplitude = amplitude  # radians
        self.frequency = frequency  # Hz
        self.duration = duration    # seconds
        self.points_per_cycle = points_per_cycle
        
        # Joint names (must match URDF and controller config)
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
        
        # Action client for JointTrajectoryController
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
        self.get_logger().info('Sine Wave Trajectory Node initialized')
        self.get_logger().info(f'  Amplitude: {self.amplitude:.2f} rad ({math.degrees(self.amplitude):.1f} deg)')
        self.get_logger().info(f'  Frequency: {self.frequency:.2f} Hz')
        self.get_logger().info(f'  Duration: {self.duration:.1f} s')

    def create_sine_trajectory(self) -> JointTrajectory:
        """Create a sine wave trajectory for all 4 motors."""
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        # Calculate number of points
        num_points = int(self.duration * self.points_per_cycle * self.frequency)
        dt = self.duration / num_points
        
        self.get_logger().info(f'Creating trajectory with {num_points} points')
        
        for i in range(num_points + 1):
            t = i * dt
            point = JointTrajectoryPoint()
            
            # Different phase for each motor (creates wave effect)
            positions = []
            velocities = []
            
            for j in range(4):
                # Phase offset: 0, 90, 180, 270 degrees
                phase = j * (math.pi / 2)
                
                # Position: A * sin(2π * f * t + phase)
                pos = self.amplitude * math.sin(2 * math.pi * self.frequency * t + phase)
                positions.append(pos)
                
                # Velocity: A * 2π * f * cos(2π * f * t + phase)
                vel = self.amplitude * 2 * math.pi * self.frequency * \
                      math.cos(2 * math.pi * self.frequency * t + phase)
                velocities.append(vel)
            
            point.positions = positions
            point.velocities = velocities
            
            # Time from start
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            
            trajectory.points.append(point)
        
        return trajectory

    def create_synchronized_trajectory(self) -> JointTrajectory:
        """Create a synchronized sine wave (all motors same phase)."""
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        num_points = int(self.duration * self.points_per_cycle * self.frequency)
        dt = self.duration / num_points
        
        self.get_logger().info(f'Creating synchronized trajectory with {num_points} points')
        
        for i in range(num_points + 1):
            t = i * dt
            point = JointTrajectoryPoint()
            
            # All motors same position (synchronized)
            pos = self.amplitude * math.sin(2 * math.pi * self.frequency * t)
            vel = self.amplitude * 2 * math.pi * self.frequency * \
                  math.cos(2 * math.pi * self.frequency * t)
            
            point.positions = [pos, pos, pos, pos]
            point.velocities = [vel, vel, vel, vel]
            
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            
            trajectory.points.append(point)
        
        return trajectory

    def create_alternating_trajectory(self) -> JointTrajectory:
        """Create alternating pattern (opposite pairs move together)."""
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        num_points = int(self.duration * self.points_per_cycle * self.frequency)
        dt = self.duration / num_points
        
        self.get_logger().info(f'Creating alternating trajectory with {num_points} points')
        
        for i in range(num_points + 1):
            t = i * dt
            point = JointTrajectoryPoint()
            
            pos = self.amplitude * math.sin(2 * math.pi * self.frequency * t)
            vel = self.amplitude * 2 * math.pi * self.frequency * \
                  math.cos(2 * math.pi * self.frequency * t)
            
            # Motors 1,4 (diagonal) together, motors 2,3 (diagonal) opposite
            point.positions = [pos, -pos, -pos, pos]
            point.velocities = [vel, -vel, -vel, vel]
            
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            
            trajectory.points.append(point)
        
        return trajectory

    def send_trajectory(self, mode: str = 'wave'):
        """Send trajectory to the controller."""
        self.get_logger().info('Waiting for action server...')
        
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            self.get_logger().error('Make sure joint_trajectory_controller is running:')
            self.get_logger().error('  ros2 launch myactuator_hardware rviz_simulation.launch.py')
            return False
        
        self.get_logger().info('Action server connected!')
        
        # Create trajectory based on mode
        if mode == 'sync':
            trajectory = self.create_synchronized_trajectory()
        elif mode == 'alternate':
            trajectory = self.create_alternating_trajectory()
        else:  # 'wave' (default)
            trajectory = self.create_sine_trajectory()
        
        # Create goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        self.get_logger().info(f'Sending {mode} trajectory...')
        
        # Send goal
        send_goal_future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        return True

    def goal_response_callback(self, future):
        """Handle goal response."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted! Executing trajectory...')
        
        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle trajectory feedback."""
        feedback = feedback_msg.feedback
        
        # Log progress periodically
        if hasattr(feedback, 'actual') and feedback.actual.positions:
            positions = feedback.actual.positions
            self.get_logger().info(
                f'Position: [{positions[0]:.2f}, {positions[1]:.2f}, '
                f'{positions[2]:.2f}, {positions[3]:.2f}] rad',
                throttle_duration_sec=1.0
            )

    def result_callback(self, future):
        """Handle trajectory result."""
        result = future.result().result
        
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Trajectory completed successfully!')
        else:
            self.get_logger().error(f'Trajectory failed with error code: {result.error_code}')
            if result.error_string:
                self.get_logger().error(f'Error: {result.error_string}')


def main():
    parser = argparse.ArgumentParser(description='Send sine wave trajectory to 4 motors')
    parser.add_argument('--amplitude', '-a', type=float, default=1.0,
                        help='Amplitude in radians (default: 1.0)')
    parser.add_argument('--frequency', '-f', type=float, default=0.3,
                        help='Frequency in Hz (default: 0.3)')
    parser.add_argument('--duration', '-d', type=float, default=10.0,
                        help='Duration in seconds (default: 10.0)')
    parser.add_argument('--mode', '-m', type=str, default='wave',
                        choices=['wave', 'sync', 'alternate'],
                        help='Trajectory mode: wave (phase shifted), sync (all same), alternate (diagonal pairs)')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    node = SineWaveTrajectoryNode(
        amplitude=args.amplitude,
        frequency=args.frequency,
        duration=args.duration
    )
    
    try:
        if node.send_trajectory(mode=args.mode):
            # Spin to process callbacks
            rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
