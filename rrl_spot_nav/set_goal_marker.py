import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose

class GoalPoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        
        # Create a subscription to the /initialpose topic
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.pose_callback,
            10)
        
        self.subscription  # prevent unused variable warning
        self.pose_data = None
        self.goal_marker_pub = self.create_publisher(Pose, '/update_marker_pose/goal', 10)
    
    def pose_callback(self, msg):
        self.get_logger().info(f'Received pose: {msg.pose.pose}')
        self.pose_data = msg.pose.pose
        self.goal_marker_pub.publish(self.pose_data)

def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseSubscriber()
    node.get_logger().info('Hello from set goal marker')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()