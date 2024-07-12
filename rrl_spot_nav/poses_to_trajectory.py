#!/usr/bin/env python3

import math
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

from spot_msgs.action import Trajectory
from gpp_action_examples_interface.action import TrajectoryToFrame
from rclpy.duration import Duration
import threading

import time

class NavigateToPoseClient(Node):

    def __init__(self):
        super().__init__('poses_to_trajectory')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.trajectory_action_client = ActionClient(self, TrajectoryToFrame, 'trajectoryToFrame')
        self.goal_done = True
        self.index = 0
        self.frame_name_prefix = "pose_"
        # Create publishers for the start and goal poses
        self.start_publisher = self.create_publisher(Pose, '/update_marker_pose/start', 10)
        self.goal_publisher = self.create_publisher(Pose, '/update_marker_pose/goal', 10)

    def lookup_transform(self, source_frame, target_frame, time, timeout=7.0):
        end_time = self.get_clock().now() + Duration(seconds=timeout)
        while self.get_clock().now() < end_time:
            try:
                transform = self.tf_buffer.lookup_transform(source_frame, target_frame, time)
                return transform
            except TransformException:
                self.get_logger().info(f'Waiting for transform {source_frame} to {target_frame}')
        return None

    def send_trajectory_goal(self, target_frame_id, duration):
        goal_msg = TrajectoryToFrame.Goal()
        goal_msg.frame_id = target_frame_id
        self.get_logger().info("wait for server")
        self.trajectory_action_client.wait_for_server()
        self.get_logger().info("Server up")
        self.trajectory_future = self.trajectory_action_client.send_goal_async(goal_msg, feedback_callback=self.trajectory_feedback_callback)
        self.trajectory_future.add_done_callback(self.trajectory_goal_response_callback)
        self.get_logger().info("Goal sended")

    def trajectory_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.goal_done = True
            return

        self.get_logger().info('Goal accepted :)')

        self.trajectory_get_result_future = goal_handle.get_result_async()
        self.trajectory_get_result_future.add_done_callback(self.trajectory_get_result_callback)

    def trajectory_get_result_callback(self, future):
        self.goal_done = True
        result = future.result().result
        self.get_logger().info(f'Trajectory Result: {result}')

    def trajectory_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

    def send_next_pose(self, frame_name):
        self.goal_done = False
        self.get_logger().info(f"Sending goal for frame_id: {frame_name}")
        self.send_trajectory_goal(frame_name, Duration(seconds=10).to_msg())

def spin_thread(client):
    rclpy.spin(client)

def main(args=None):
    print("Hello from navigate_to_frame_client Node")
    rclpy.init(args=args)
    client = NavigateToPoseClient()
    #rclpy.spin_once(client)
    frame_names = []
    
    thread = threading.Thread(target=spin_thread, args=(client,))
    thread.start()
    index = 0
    transforms = []
    while True:
        frame_name = f'{client.frame_name_prefix}{index}'
        transform = client.lookup_transform('map', frame_name, rclpy.time.Time())
        if transform is None:
            client.get_logger().info('No frames to store')
            break
        #if not index == 0:
        frame_names.append(frame_name)
        transforms.append(transform)
        index +=1
    
    start_tf = transforms[0]
    end_tf = transforms[-1]

    end_pose = Pose()
    end_pose.position = Point(
            x=start_tf.transform.translation.x,
            y=start_tf.transform.translation.y,
            z=start_tf.transform.translation.z
    )
    end_pose.orientation = Quaternion(
            x=start_tf.transform.rotation.x,
            y=start_tf.transform.rotation.y,
            z=start_tf.transform.rotation.z,
            w=start_tf.transform.rotation.w
    )

    start_pose = Pose()
    start_pose.position = Point(
        x=end_tf.transform.translation.x,
        y=end_tf.transform.translation.y,
        z=end_tf.transform.translation.z+0.1
    )
    start_pose.orientation = Quaternion(
        x=end_tf.transform.rotation.x,
        y=end_tf.transform.rotation.y,
        z=end_tf.transform.rotation.z,
        w=end_tf.transform.rotation.w
    )


    client.get_logger().info('Start and goal poses published')


    # index = 0

    for frame_name in frame_names:
        print(frame_name)
        client.send_next_pose(frame_name)

        # self.goal_done set to false in send_next_pose and true in trajectory_get_result_callback
        while not client.goal_done:    
            client.get_logger().info('wait')
            time.sleep(0.5)
        # index +=1
    print("All poses send")
    client.get_logger().info('Swapping start and end marker')

    # New End is Start

    # Publish start and end poses
    client.start_publisher.publish(start_pose)
    client.goal_publisher.publish(end_pose)

    rclpy.shutdown()
    thread.join()

if __name__ == '__main__':
    main()
