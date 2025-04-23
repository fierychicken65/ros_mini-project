import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import select
import tty
import termios
import time

# --- Configuration ---
LIN_VEL_STEP_SIZE = 20.0  # Speed for forward/backward
TURN_VEL = 150.0         # FIXED speed for wheels during turns (Adjust if too fast/slow)
MAX_VEL = 200.0          # Maximum velocity for any wheel

# Define key mappings
KEY_FORWARD = 'w'
KEY_BACKWARD = 's'
KEY_LEFT = 'a'
KEY_RIGHT = 'd'
KEY_STOP = 'q'
KEY_QUIT = '\x1b' # Escape key

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        self.settings = termios.tcgetattr(sys.stdin)
        self.wheel_velocities_pub = self.create_publisher(
            Float64MultiArray,
            '/velocity_controller/commands', # MAKE SURE THIS TOPIC IS CORRECT!
            10)
        self.get_logger().info("KeyboardControlNode initialized.")

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
Control Your Robot! (Quick Fix Attempt)
---------------------------
Moving around:
      {}
  {}    {}    {}

{} : stop
ESC to quit

Watch terminal for 'Sending Velocities:' logs!
        """.format(KEY_FORWARD, KEY_LEFT, KEY_BACKWARD, KEY_RIGHT, KEY_STOP)
        self.get_logger().info(self.msg)

        target_left_vel = 0.0
        target_right_vel = 0.0

        try:
            while rclpy.ok():
                key = self.getKey()
                action_taken = True # Flag to check if we should publish

                if key == KEY_FORWARD:
                    target_left_vel = LIN_VEL_STEP_SIZE
                    target_right_vel = LIN_VEL_STEP_SIZE
                elif key == KEY_BACKWARD:
                    target_left_vel = -LIN_VEL_STEP_SIZE
                    target_right_vel = -LIN_VEL_STEP_SIZE
                elif key == KEY_LEFT: # Turn left in place - FIXED VELOCITY
                    target_left_vel = -TURN_VEL # Left wheel backward
                    target_right_vel = TURN_VEL  # Right wheel forward
                elif key == KEY_RIGHT: # Turn right in place - FIXED VELOCITY
                    target_left_vel = TURN_VEL   # Left wheel forward
                    target_right_vel = -TURN_VEL # Right wheel backward
                elif key == KEY_STOP:
                    target_left_vel = 0.0
                    target_right_vel = 0.0
                elif key == KEY_QUIT:
                    self.get_logger().info("Escape key pressed. Exiting.")
                    break
                elif key == '': # If no key was pressed, stop
                    target_left_vel = 0.0
                    target_right_vel = 0.0
                else: # Ignore other keys
                    action_taken = False # No relevant key pressed
                    continue # Skip publish for irrelevant keys

                # --- Apply Velocity Limits ---
                current_left_vel = max(-MAX_VEL, min(target_left_vel, MAX_VEL))
                current_right_vel = max(-MAX_VEL, min(target_right_vel, MAX_VEL))

                # --- Prepare Message Data ---
                # !! VERY IMPORTANT !!: Does your controller expect [L, R] or [L, R, L, R]?
                # Option 1: [Left, Right]
                # wheel_data = [current_left_vel, current_right_vel]
                # Option 2: [Left, Right, Left, Right] (or maybe [FL, FR, RL, RR]?)
                wheel_data = [current_left_vel, current_right_vel, current_left_vel, current_right_vel]

                # --- **** Log the command being sent **** ---
                # Check this output carefully when pressing keys!
                self.get_logger().info(f"Key: '{key}', Sending Velocities: {wheel_data}")

                # --- Publish Velocities ---
                wheel_velocities = Float64MultiArray()
                wheel_velocities.data = wheel_data
                self.wheel_velocities_pub.publish(wheel_velocities)

                # time.sleep(0.02) # Optional small delay

        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")
            import traceback
            traceback.print_exc() # Print detailed error traceback
        finally:
            # --- Ensure robot stops and terminal is restored on exit ---
            self.get_logger().info("Stopping robot and restoring terminal settings.")
            stop_velocities = Float64MultiArray()
             # Use the same format as above ([L,R] or [L,R,L,R])
            stop_velocities.data = [0.0, 0.0, 0.0, 0.0] # Adjust if needed
            # Check if publisher was created successfully before using it
            if hasattr(self, 'wheel_velocities_pub') and self.wheel_velocities_pub:
                 try:
                     # Add a small delay and potentially publish multiple times to ensure it gets through
                     time.sleep(0.1)
                     self.wheel_velocities_pub.publish(stop_velocities)
                     time.sleep(0.1)
                     self.wheel_velocities_pub.publish(stop_velocities)
                 except Exception as pub_e:
                     self.get_logger().error(f"Error publishing stop command: {pub_e}")

            # Restore terminal unconditionally
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            self.get_logger().info("Terminal settings restored.")


def main(args=None):
    rclpy.init(args=args)
    node = None # Initialize to None
    try:
        node = KeyboardControlNode()
        node.run_keyboard_control()
    except Exception as main_e:
        if node:
            node.get_logger().error(f"Unhandled exception in main: {main_e}")
        else:
            print(f"Unhandled exception before node creation: {main_e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        if node:
            node.get_logger().info("Shutting down node.")
            node.destroy_node()
        rclpy.shutdown()
        print("rclpy shutdown complete.")

if __name__ == '__main__':
    main()