import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped, Point, Quaternion
from tf2_ros import TransformException
from rclpy.duration import Duration

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import threading

from interactive_markers import InteractiveMarkerServer

from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker

class GoalMarkerPublisher(Node):
    def __init__(self):
        super().__init__('set_start_marker')
        # self.publisher_ = self.create_publisher(Pose, 'interactive_marker_point_xyz_node/out/pose3d/pose3d/start', 10)
        self.publisher_ = self.create_publisher(Pose, 'update_marker_pose/start', 10)
        #self.timer = self.create_timer(1.0, self.timer_callback)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def set_marker_to_body(self):
        end_time = self.get_clock().now() + Duration(seconds=10.0)
        transformstamped = TransformStamped()
        while self.get_clock().now() < end_time:
            try:
                transformstamped = self.tf_buffer.lookup_transform('map', 'body', rclpy.time.Time())
                #return transformstamped
            except TransformException:
                self.get_logger().info(f'Waiting for transform map to body')
        
        if transformstamped is None:
            self.get_logger().info(f'Transform is None')
            return
            # create an interactive marker server on the namespace simple_marker
        
        msg = Pose()
        # Set your pose values here
        msg.position.x = transformstamped.transform.translation.x
        msg.position.y = transformstamped.transform.translation.y
        msg.position.z = 1.0
        msg.orientation.x = transformstamped.transform.rotation.x
        msg.orientation.y = transformstamped.transform.rotation.y
        msg.orientation.z = transformstamped.transform.rotation.z
        msg.orientation.w = transformstamped.transform.rotation.w

        new_pose = Pose()
        new_pose.position = Point(x=transformstamped.transform.translation.x, y=transformstamped.transform.translation.y, z=transformstamped.transform.translation.z)  # Update the coordinates as needed
        new_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.publisher_.publish(new_pose)
        self.get_logger().info('Publishing goal: "%s"' % msg)


def spin_thread(goal_marker_pub):
    rclpy.spin(goal_marker_pub)

def main(args=None):
    rclpy.init(args=args)
    goal_marker_pub = GoalMarkerPublisher()

    thread = threading.Thread(target=spin_thread, args=(goal_marker_pub,))
    thread.start()

    goal_marker_pub.set_marker_to_body()
    goal_marker_pub.get_logger().info('Destroy node')
    # Destroy the node explicitly
    goal_marker_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()