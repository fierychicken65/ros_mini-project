 #!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import sys
import select
import tty
import termios
# from pynput import keyboard

# Define key codes
STEP_SIZE = 0.01
# ANG_VEL_STEP_SIZE = 0.1

class KeyboardControlNode(Node):

    def __init__(self):
        super().__init__('keyboard_control_node')

        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        # self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.settings = termios.tcgetattr(sys.stdin)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_keyboard_control(self):
        self.msg = """
        Control Your Arm
        
        """

        self.get_logger().info(self.msg)
        joint_positions = Float64MultiArray()
        # wheel_velocities = Float64MultiArray()
        
        q1 = 0.0
        q2 = 0.0        
        q3 = 0.0        
        q4 = 0.0
        q5 = 0.0

        while True:
            key = self.getKey()
            if key is not None:
                if key == '\x1b':  # Escape key
                    break
                elif key == 'q':  # Quit
                    
                    q1=0.0
                    q2=0.0
                    q3=0.0
                    q4=0.0
                    q5=0.0
                elif key == 'z':
                    q1 += STEP_SIZE
                elif key == 'x':
                    q2 += STEP_SIZE
                elif key == 'c':
                    q3 += STEP_SIZE
                elif key == 'v':
                    q4 += STEP_SIZE
                elif key == 'b':
                    q5 += STEP_SIZE
                elif key == 'g':
                    q1 -= STEP_SIZE
                elif key == 'h':
                    q2 -= STEP_SIZE
                elif key == 'j':
                    q3 -= STEP_SIZE
                elif key == 'k':
                    q4 -= STEP_SIZE
                elif key == 'l':
                    q5 -= STEP_SIZE
                    
                # elif key == 'w':  # Forward
                #     linear_vel += LIN_VEL_STEP_SIZE
                # elif key == 's':  # Reverse
                #     linear_vel -= LIN_VEL_STEP_SIZE
                # elif key == 'd':  # Right
                #     steer_angle -= ANG_VEL_STEP_SIZE
                # elif key == 'a':  # Left
                #     steer_angle += ANG_VEL_STEP_SIZE


                # if angle>2.4:
                #     angle=1.0
                # if angle<-1.0:
                #     angle=-1.0

                print("q1: ",q1, "; q2: ",q2,"; q3: ",q3,"; q4: ",q4,"; q5: ",q5, "\n")
                joint_positions.data = [q1,q2,q3,q4,q5]

                self.joint_position_pub.publish(joint_positions)
                # self.wheel_velocities_pub.publish(wheel_velocities)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run_keyboard_control()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()