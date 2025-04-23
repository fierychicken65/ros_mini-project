from sympy import *
import sympy as sp
import numpy as np
from scipy import linalg
import scipy as scpy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node
import sys
import select
import tty
import termios
from numpy.linalg import pinv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import PoseStamped
import time

def dh_Trans(q):
    # Converted DH parameters to meters
    T_0_1 = Matrix([
        [cos(q[0]), -sin(q[0])*cos(pi/2), sin(q[0])*sin(pi/2), 0],
        [sin(q[0]), cos(q[0])*cos(pi/2), -cos(q[0])*sin(pi/2), 0],
        [0, sin(pi/2), cos(pi/2), 0.150],  # 150 mm -> 0.15 m
        [0, 0, 0, 1]
    ])
    
    T_1_2 = Matrix([
        [cos(q[1]+pi/2), -sin(q[1]+pi/2)*cos(0), sin(q[1]+pi/2)*sin(0), 0.150*cos(q[1]+pi/2)],  # 150 mm
        [sin(q[1]+pi/2), cos(q[1]+pi/2)*cos(0), -cos(q[1]+pi/2)*sin(0), 0.150*sin(q[1]+pi/2)],
        [0, sin(0), cos(0), -0.00943],  # -9.43 mm
        [0, 0, 0, 1]
    ])
    
    T_2_3 = Matrix([
        [cos(q[2]), -sin(q[2])*cos(0), sin(q[2])*sin(0), 0.100*cos(q[2])],  # 100 mm
        [sin(q[2]), cos(q[2])*cos(0), -cos(q[2])*sin(0), 0.100*sin(q[2])],
        [0, sin(0), cos(0), 0],
        [0, 0, 0, 1]
    ])
    
    T_3_4 = Matrix([
        [cos(q[3]-pi/2), -sin(q[3]-pi/2)*cos(-pi/2), sin(q[3]-pi/2)*sin(-pi/2), 0],
        [sin(q[3]-pi/2), cos(q[3]-pi/2)*cos(-pi/2), -cos(q[3]-pi/2)*sin(-pi/2), 0],
        [0, sin(-pi/2), cos(-pi/2), 0],
        [0, 0, 0, 1]
    ])
    
    T_4_5 = Matrix([
        [cos(q[4]), -sin(q[4])*cos(0), sin(q[4])*sin(0), 0],
        [sin(q[4]), cos(q[4])*cos(0), -cos(q[4])*sin(0), 0],
        [0, sin(0), cos(0), 0.119],  # 119 mm
        [0, 0, 0, 1]
    ])
    
    return T_0_1*T_1_2*T_2_3*T_3_4*T_4_5

def dh_Jacobian(q):
    # Converted DH parameters to meters (same as dh_Trans)
    T_0_1 = Matrix([
        [cos(q[0]), -sin(q[0])*cos(pi/2), sin(q[0])*sin(pi/2), 0],
        [sin(q[0]), cos(q[0])*cos(pi/2), -cos(q[0])*sin(pi/2), 0],
        [0, sin(pi/2), cos(pi/2), 0.150],
        [0, 0, 0, 1]
    ])
    
    T_1_2 = Matrix([
        [cos(q[1]+pi/2), -sin(q[1]+pi/2)*cos(0), sin(q[1]+pi/2)*sin(0), 0.150*cos(q[1]+pi/2)],
        [sin(q[1]+pi/2), cos(q[1]+pi/2)*cos(0), -cos(q[1]+pi/2)*sin(0), 0.150*sin(q[1]+pi/2)],
        [0, sin(0), cos(0), -0.00943],
        [0, 0, 0, 1]
    ])
    
    T_2_3 = Matrix([
        [cos(q[2]), -sin(q[2])*cos(0), sin(q[2])*sin(0), 0.100*cos(q[2])],
        [sin(q[2]), cos(q[2])*cos(0), -cos(q[2])*sin(0), 0.100*sin(q[2])],
        [0, sin(0), cos(0), 0],
        [0, 0, 0, 1]
    ])
    
    T_3_4 = Matrix([
        [cos(q[3]-pi/2), -sin(q[3]-pi/2)*cos(-pi/2), sin(q[3]-pi/2)*sin(-pi/2), 0],
        [sin(q[3]-pi/2), cos(q[3]-pi/2)*cos(-pi/2), -cos(q[3]-pi/2)*sin(-pi/2), 0],
        [0, sin(-pi/2), cos(-pi/2), 0],
        [0, 0, 0, 1]
    ])
    
    T_4_5 = Matrix([
        [cos(q[4]), -sin(q[4])*cos(0), sin(q[4])*sin(0), 0],
        [sin(q[4]), cos(q[4])*cos(0), -cos(q[4])*sin(0), 0],
        [0, sin(0), cos(0), 0.119],
        [0, 0, 0, 1]
    ])
    
    return [
        T_0_1,
        T_0_1*T_1_2,
        T_0_1*T_1_2*T_2_3,
        T_0_1*T_1_2*T_2_3*T_3_4,
        T_0_1*T_1_2*T_2_3*T_3_4*T_4_5
    ]

def cross(M1, M2):
    c = [M1[1] * M2[2] - M1[2] * M2[1],
         M1[2] * M2[0] - M1[0] * M2[2],
         M1[0] * M2[1] - M1[1] * M2[0]]
    return c

def jacobian(T):
    z = [sp.Matrix([0,0,1])]
    for i in T:
        z.append((i[:3,2]))

    o = [sp.Matrix([0,0,0])]
    for i in T:
        o.append((i[:3,3]))

    J = sp.zeros(6, 5)

    for i in range(5):
        J[0:3, i] = sp.Matrix(cross(z[i], 
            [o[-1][0] - o[i][0], 
            o[-1][1] - o[i][1], 
            o[-1][2] - o[i][2]]))

    for i in range(5):
        J[3:6, i] = z[i]
    
    return J

class ArmControlNode(Node):
    def __init__(self):
        super().__init__('arm_control_node')
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.subscription = self.create_subscription(
            PoseStamped, '/odom', self.odom_callback, 10)
        self.ef_x = 0.0
        self.ef_y = 0.0
        self.ef_z = 0.0
        self.odom_received = False # Add a flag
        self.subscription = self.create_subscription(
            PoseStamped, '/odom', self.odom_callback, 10)
        self.get_logger().info('ArmControlNode initialized.')

    def odom_callback(self, msg):
        self.ef_x = msg.pose.position.x
        self.ef_y = msg.pose.position.y
        self.ef_z = msg.pose.position.z
        self.odom_received = True # Set flag on first message

    def run_arm_control(self):
        # --- Wait for the first Odom message ---
        self.get_logger().info('Waiting for first odom message...')
        while rclpy.ok() and not self.odom_received:
            rclpy.spin_once(self, timeout_sec=0.1) # Process callbacks
            time.sleep(0.05) # Prevent busy-waiting

        if not rclpy.ok():
            return # Exit if ROS shutdown during wait

        self.get_logger().info(f'Initial odom received: {[self.ef_x, self.ef_y, self.ef_z]}')
        # -----------------------------------------

        # Define initial q guess (as before or maybe read from /joint_states later)
        initial_q_guess = sp.Matrix([0.0, 1.57, 0.0, -1.57, 0.0])

        # Define the nested inv_kine function (as before)
        def inv_kine(q):
            target_position = input("Enter target position (X Y Z in meters): ")
            target_position_vector = np.array([float(num) for num in target_position.split()])
            
            present_ef_pos = [self.ef_x, self.ef_y, self.ef_z]
            print(f"Current position: {present_ef_pos}")
            print(f"Target position: {target_position_vector}")

            total_time_to_move = 5.0
            dt = 0.001
            time_step = np.arange(0, total_time_to_move, dt)

            fig = plt.figure()
            ax = fig.add_subplot(111, projection='3d')
            x_points, y_points, z_points = [], [], []

            for i in time_step:
                X_dot_bar = np.array([
                    (target_position_vector[0] - present_ef_pos[0])/total_time_to_move,
                    (target_position_vector[1] - present_ef_pos[1])/total_time_to_move,
                    (target_position_vector[2] - present_ef_pos[2])/total_time_to_move,
                    0, 0, 0
                ])
                
                T = dh_Jacobian(q)
                J = jacobian(T)
                J_inv = pinv(np.array(J).astype(float))
                
                q_dot = np.dot(J_inv, X_dot_bar)
                q = q + sp.Matrix(q_dot * dt)
                
                joint_positions = Float64MultiArray()
                joint_positions.data = [float(q[i]) for i in range(5)]
                self.joint_position_pub.publish(joint_positions)

                T_ef = dh_Trans(q)
                x, y, z = float(T_ef[0,3]), float(T_ef[1,3]), float(T_ef[2,3])
                x_points.append(x)
                y_points.append(y)
                z_points.append(z)
                print(f"X: {x:.3f}m, Y: {y:.3f}m, Z: {z:.3f}m")

            ax.plot3D(x_points, y_points, z_points)
            ax.set_title("End-Effector Path")
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            plt.show()

        inv_kine(initial_q_guess)

def main(args=None):
    rclpy.init(args=args)
    node = ArmControlNode()
    node.run_arm_control()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()