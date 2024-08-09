import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.duration import Duration
from tf2_ros import TransformException
from std_srvs.srv import Trigger

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
        self.start_marker_pub = self.create_publisher(Pose, '/update_marker_pose/start', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.plan_path_client = self.create_client(Trigger, 'plan_path')
        while not self.plan_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for plan_path_service to be available...')

        # Call once to initialize Marker Array.
        # ToDo: Set start and goal from Octree not MarkerArray to get rid of Marker
        self.call_plan_path_service()

    def pose_callback(self, msg):
        #self.get_logger().info(f'Received pose: {msg.pose.pose}')
        self.pose_data = msg.pose.pose

        # Send start and end to planner
        self.get_logger().info('Send start and end.')
        self.goal_marker_pub.publish(self.pose_data)
        self.set_start_marker_pose()

        # Plan before and then publish start and end
        #self.call_plan_path_service()

    def call_plan_path_service(self):
        req = Trigger.Request()
        future = self.plan_path_client.call_async(req)
        future.add_done_callback(self.plan_path_response_callback)

    def plan_path_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Path planning succeeded. Publishing goal marker.')
                self.goal_marker_pub.publish(self.pose_data)
                self.set_start_marker_pose()  # Call the method to set the start marker pose
            else:
                self.get_logger().error('Path planning failed: ' + response.message)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def set_start_marker_pose(self):
        end_time = self.get_clock().now() + Duration(seconds=10.0)
        transformstamped = TransformStamped()
        while self.get_clock().now() < end_time:
            try:
                transformstamped = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                break
            except TransformException:
                self.get_logger().info('Waiting for transform map to base_footprint')

        if transformstamped is None:
            self.get_logger().info('Transform is None')
            return

        start_pose = Pose()
        start_pose.position.x = transformstamped.transform.translation.x
        start_pose.position.y = transformstamped.transform.translation.y
        start_pose.position.z = transformstamped.transform.translation.z
        start_pose.orientation.x = transformstamped.transform.rotation.x
        start_pose.orientation.y = transformstamped.transform.rotation.y
        start_pose.orientation.z = transformstamped.transform.rotation.z
        start_pose.orientation.w = transformstamped.transform.rotation.w

        self.start_marker_pub.publish(start_pose)
        self.get_logger().info('Publishing start marker pose')

def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseSubscriber()
    node.get_logger().info('Hello from set goal marker')

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
