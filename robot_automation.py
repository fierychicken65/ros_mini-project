#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class RobotAutomationNode(Node):
    def __init__(self):
        super().__init__('robot_automation_node')
        
        # Create publishers for both robot movement and arm control
        self.wheel_velocities_pub = self.create_publisher(
            Float64MultiArray, 
            '/velocity_controller/commands', 
            10)
        self.joint_position_pub = self.create_publisher(
            Float64MultiArray,
            '/position_controller/commands',
            10)
        
        # Initialize joint positions
        self.current_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0]

    def move_robot(self, left_vel, right_vel, duration):
        """Move the robot with specified wheel velocities for a duration"""
        wheel_velocities = Float64MultiArray()
        wheel_velocities.data = [left_vel, right_vel, left_vel, right_vel]
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.wheel_velocities_pub.publish(wheel_velocities)
            time.sleep(0.1)  # Control rate
        
        # Stop the robot
        wheel_velocities.data = [0.0, 0.0, 0.0, 0.0]
        self.wheel_velocities_pub.publish(wheel_velocities)

    def move_joint(self, joint_index, direction, duration):
        """Move a specific joint in a direction for a duration"""
        # Set the step size for joint movement
        STEP_SIZE = 0.1
        
        start_time = time.time()
        while time.time() - start_time < duration:
            # Update the specific joint position
            self.current_joint_positions[joint_index] += STEP_SIZE * direction
            
            # Create and publish joint positions message
            joint_positions = Float64MultiArray()
            joint_positions.data = self.current_joint_positions.copy()
            self.joint_position_pub.publish(joint_positions)
            
            time.sleep(0.1)  # Control rate

    def run_automation(self):
        """Run a sequence of automated movements"""
        # First sequence
        # Move forward for 2.5 seconds
        self.move_robot(20.0, 20.0, 5)  # Forward at speed 20.0
        
        # Move joint 2 (x key) for 0.5 seconds
        self.move_joint(1, 1, 0.5)  # Joint 2 (index 1), positive direction
        
        # Move joint 3 (j key) for 0.5 seconds
        self.move_joint(2, -1, 0.5)  # Joint 3 (index 2), negative direction
        
        # Pause for 2 seconds
        time.sleep(2.0)
        
        # Move backward for 2.5 seconds
        self.move_robot(-20.0, -20.0, 2.5)  # Backward at speed -20.0
        
        # Second sequence (mirrored)
        # Move joint 3 back for 0.5 seconds
        self.move_joint(2, 1, 0.5)  # Joint 3 (index 2), positive direction (reverse)
        
        # Move joint 2 back for 0.5 seconds
        self.move_joint(1, -1, 0.5)  # Joint 2 (index 1), negative direction (reverse)
        
        # Move joint 2 back again for 0.5 seconds
        self.move_joint(1, -1, 0.5)  # Joint 2 (index 1), negative direction (reverse)
        
        # Move joint 3 back for 0.5 seconds
        self.move_joint(2, 1, 0.5)  # Joint 3 (index 2), positive direction (reverse)

def main(args=None):
    rclpy.init(args=args)
    node = RobotAutomationNode()
    
    try:
        node.run_automation()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 