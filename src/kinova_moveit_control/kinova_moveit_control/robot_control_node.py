import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np

class KinovaControlNode(Node):
    def __init__(self):
        super().__init__('kinova_control_node')
        
        # Publisher for joint trajectory commands
        self.trajectory_publisher = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        # Subscriber to get current joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Store current joint positions
        self.current_joint_positions = None
        
        # Joint names for Kinova Gen3
        self.joint_names = [
            'joint_1', 
            'joint_2', 
            'joint_3', 
            'joint_4', 
            'joint_5', 
            'joint_6',
            'joint_7'
        ]
        
        # Create a timer to demonstrate movement (optional)
        self.timer = self.create_timer(5.0, self.move_robot)

    def joint_state_callback(self, msg):
        """
        Callback to update current joint positions
        """
        self.current_joint_positions = msg.position
        self.get_logger().info(f'Current Joint Positions: {self.current_joint_positions}')

    def move_to_joint_positions(self, joint_positions):
        """
        Move the robot to specified joint positions.
        
        :param joint_positions: List of target joint angles in radians
        """
        # Validate input
        if len(joint_positions) != len(self.joint_names):
            self.get_logger().error(f'Invalid number of joint positions. Expected {len(self.joint_names)}, got {len(joint_positions)}')
            return
        
        # Create joint trajectory message
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = self.joint_names
        
        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        
        # Set movement parameters
        point.time_from_start.sec = 2  # 2 seconds to reach position
        point.velocities = [0.5] * len(self.joint_names)
        point.accelerations = [0.1] * len(self.joint_names)
        
        # Add point to trajectory
        trajectory.points.append(point)
        
        # Publish trajectory
        self.get_logger().info(f'Moving to joint positions: {joint_positions}')
        self.trajectory_publisher.publish(trajectory)

    def move_robot(self):
        """
        Example method to demonstrate robot movement.
        Modify this with your specific movement requirements.
        """
        # Check if we have current joint positions
        if self.current_joint_positions is None:
            self.get_logger().warn('No current joint positions available. Waiting...')
            return
        
        # Predefined joint positions
        positions_list = [
            # Home/zero position
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            
            # Alternative positions (adjust as needed)
            [np.pi/4, -np.pi/4, np.pi/4, -np.pi/4, np.pi/4, 0.0, 0.0],
            [0.5, 0.2, -0.3, 0.1, 0.2, -0.1, 0.0],
            #[1, 0.5, -0.5, 0.3, 0.4, -0.5],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [np.pi/4, -np.pi/4, np.pi/4, -np.pi/4, np.pi/4, 0.0, 0.0],
            [0.5, 0.2, -0.3, 0.1, 0.2, -0.1, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ]
        
        # Cycle through predefined positions
        for positions in positions_list:
            self.move_to_joint_positions(positions)
            rclpy.spin_once(self, timeout_sec=3.0)  # Allow time for movement

def main(args=None):
    rclpy.init(args=args)
    node = KinovaControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


"""
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
  joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
  points: [
    { positions: [0, 0, 0, 0, 0, 0, 0], time_from_start: { sec: 10 } },
  ]
}" -1
 https://wiki.ros.org/ros_controllers_cartesian
"""