import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

class KinovaMonitorNode(Node):
    def __init__(self):
        super().__init__('kinova_monitor_node')
        
        # Subscriber to get current joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Subscriber to get end-effector pose
        self.ee_pose_subscriber = self.create_subscription(
            PoseStamped,
            '/end_effector_pose',  # Change this if needed
            self.ee_pose_callback,
            10
        )
    
    def joint_state_callback(self, msg):
        """
        Callback to print current joint positions
        """
        self.get_logger().info(f'Current Joint Positions: {msg.position}')
    
    def ee_pose_callback(self, msg):
        """
        Callback to print current end-effector position
        """
        self.get_logger().info(f'Current End-Effector Position: {msg.pose.position}')


def main(args=None):
    rclpy.init(args=args)
    node = KinovaMonitorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
