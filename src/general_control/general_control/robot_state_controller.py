import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String # To publish the current state

# Define states for clarity
class RobotState:
    LANE_FOLLOWING = "lane_following"
    STOP_SIGN_ALGORITHM = "stop_sign_algorithm"
    # Add other states as needed, e.g., EMERGENCY_STOP, TRAFFIC_LIGHT_WAIT, etc.

class RobotStateController(Node):
    def __init__(self):
        super().__init__('robot_state_controller')

        self.get_logger().info("Robot State Controller Node Initialized.")

        # Declare parameters for topics if needed, or use hardcoded for simplicity
        self.declare_parameter('bus_stop_signal_topic', '/bus_stop_detected_signal')
        self.declare_parameter('stop_sign_algorithm_completion_topic', '/stop_sign_algorithm_completed')
        self.declare_parameter('robot_current_state_topic', '/robot_current_state')

        self.bus_stop_signal_topic = self.get_parameter('bus_stop_signal_topic').get_parameter_value().string_value
        self.stop_sign_algorithm_completion_topic = self.get_parameter('stop_sign_algorithm_completion_topic').get_parameter_value().string_value
        self.robot_current_state_topic = self.get_parameter('robot_current_state_topic').get_parameter_value().string_value

        # Current state of the robot
        self.current_state = RobotState.LANE_FOLLOWING
        self.get_logger().info(f"Initial State: {self.current_state}")

        # Publishers
        self.state_publisher = self.create_publisher(String, self.robot_current_state_topic, 10)
        self.publish_current_state() # Publish initial state

        # Subscribers
        self.create_subscription(
            Bool,
            self.bus_stop_signal_topic,
            self.bus_stop_signal_callback,
            10
        )

        self.create_subscription(
            Bool,
            self.stop_sign_algorithm_completion_topic,
            self.stop_sign_algorithm_completion_callback,
            10
        )

        self.get_logger().info(f"Subscribing to '{self.bus_stop_signal_topic}' for bus stop signals.")
        self.get_logger().info(f"Subscribing to '{self.stop_sign_algorithm_completion_topic}' for algorithm completion.")
        self.get_logger().info(f"Publishing robot state to '{self.robot_current_state_topic}'.")

    def publish_current_state(self):
        msg = String()
        msg.data = self.current_state
        self.state_publisher.publish(msg)
        self.get_logger().info(f"Published State: {self.current_state}")

    def bus_stop_signal_callback(self, msg):
        # Trigger condition: Bus stop detected and we are currently in lane following
        if msg.data is True and self.current_state == RobotState.LANE_FOLLOWING:
            self.get_logger().info("Bus stop signal received (True). Transitioning to STOP_SIGN_ALGORITHM state.")
            self.current_state = RobotState.STOP_SIGN_ALGORITHM
            self.publish_current_state()
        # If msg.data is False, it means the sign is no longer detected,
        # but the algorithm should finish its course.
        # The transition back to LANE_FOLLOWING will be handled by the stop_sign_algorithm_completion_callback.

    def stop_sign_algorithm_completion_callback(self, msg):
        # Trigger condition: Stop sign algorithm completed and we are currently in that state
        if msg.data is True and self.current_state == RobotState.STOP_SIGN_ALGORITHM:
            self.get_logger().info("Stop Sign Algorithm completion signal received (True). Transitioning to LANE_FOLLOWING state.")
            self.current_state = RobotState.LANE_FOLLOWING
            self.publish_current_state()

def main(args=None):
    rclpy.init(args=args)
    node = RobotStateController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()