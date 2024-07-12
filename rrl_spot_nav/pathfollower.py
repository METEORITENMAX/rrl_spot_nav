import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from tf_transformations import euler_from_quaternion
import tf2_ros
import math

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        self.declare_parameter('path_topic', '/astar_planner_node/out/localPath')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('robot_frame', 'body')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('map_frame', 'map')

        self.path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value

        self.path_subscriber = self.create_subscription(Path, self.path_topic, self.path_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, self.cmd_vel_topic, 1)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.path = None
        self.current_index = 0

        self.timer = self.create_timer(0.1, self.follow_path)
        self.get_logger().info("Timer created to follow path")

    def path_callback(self, msg):
        self.path = msg
        self.current_index = 0
        self.get_logger().info(f"Received new path with {len(msg.poses)} poses")

    def follow_path(self):
        self.get_logger().info("follow_path called")
        if not self.path:
            self.get_logger().info("No path available")
            return
        
        if self.current_index >= len(self.path.poses):
            self.get_logger().info("End of path reached")
            return
        
        try:
            trans = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rclpy.time.Time())
            current_pose = trans.transform
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Failed to get transform from {self.odom_frame} to {self.robot_frame}: {str(e)}')
            return

        target_pose = self.path.poses[self.current_index]

        cmd_vel = self.calculate_velocity_command(current_pose, target_pose.pose)
        self.velocity_publisher.publish(cmd_vel)
        self.get_logger().info(f"Published velocity command: {cmd_vel}")

        if self.is_target_reached(current_pose, target_pose.pose):
            self.current_index += 1
            self.get_logger().info(f"Moving to next target: {self.current_index}")

    def calculate_velocity_command(self, current_pose, target_pose):
        cmd_vel = Twist()

        dx = target_pose.position.x - current_pose.translation.x
        dy = target_pose.position.y - current_pose.translation.y

        distance = math.sqrt(dx**2 + dy**2)
        angle_to_target = math.atan2(dy, dx)

        current_yaw = euler_from_quaternion([
            current_pose.rotation.x,
            current_pose.rotation.y,
            current_pose.rotation.z,
            current_pose.rotation.w
        ])[2]

        angle_difference = angle_to_target - current_yaw

        kp_linear = 1.0
        kp_angular = 2.0

        cmd_vel.linear.x = kp_linear * distance
        cmd_vel.angular.z = kp_angular * angle_difference

        # Clamp the velocities to not exceed 1.0
        max_linear_velocity = 1.0
        max_angular_velocity = 1.0

        cmd_vel.linear.x = max(-max_linear_velocity, min(cmd_vel.linear.x, max_linear_velocity))
        cmd_vel.angular.z = max(-max_angular_velocity, min(cmd_vel.angular.z, max_angular_velocity))

        return cmd_vel

    def is_target_reached(self, current_pose, target_pose):
        distance_threshold = 0.1
        dx = target_pose.position.x - current_pose.translation.x
        dy = target_pose.position.y - current_pose.translation.y

        return math.sqrt(dx**2 + dy**2) < distance_threshold

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()