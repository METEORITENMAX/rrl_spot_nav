#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Pose, Twist, PoseStamped, Quaternion
import numpy as np
import tf2_ros
from copy import deepcopy
from std_srvs.srv import SetBool


import heapq
import math
import scipy.interpolate as si
import numpy as np


SPEED = 0.4
LOOKAHEAD_DISTANCE = 0.4
TARGET_ERROR = 0.4


class SimplePurePursuit(Node):
    def __init__(self):
        super().__init__('detector')

        self.declare_parameter('path_topic', '/simple_path')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')

        self.path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value

        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.current_path_world = []
        self.path_subscriber = self.create_subscription(Path, self.path_topic, self.path_callback, 10)

        # Initialize the transform buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)


        self.goal_pose_sub_ = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10)

        self.path_publisher = self.create_publisher(Path, '/simple_path', 10)

        self.create_timer(0.1, self.path_follower_timer_callback)

        self.srv = self.create_service(SetBool, '/navigation/follow_path', self.set_in_motion_callback)

        self.in_motion = False
        self.pursuit_index = 0

    def goal_pose_callback(self, msg):
        self.get_logger().info(f'Received goal pose: {msg}')

        try:
            # Lookup transform from odom to base_link
            transform = self.tf_buffer.lookup_transform(self.odom_frame, self.robot_frame, rclpy.time.Time())

            # Extract the position from the transform
            start_position = Point(
                x=transform.transform.translation.x,
                y=transform.transform.translation.y,
                z=transform.transform.translation.z
            )

            goal_pose = msg.pose

            path = Path()
            path.header.stamp = self.get_clock().now().to_msg()
            path.header.frame_id = self.odom_frame

            start_pose = PoseStamped()
            start_pose.header.stamp = self.get_clock().now().to_msg()
            start_pose.header.frame_id = self.robot_frame
            start_pose.pose = Pose(position=Point(x=start_position.x, y=start_position.y, z=start_position.z), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))

            goal_pose_stamped = PoseStamped()
            goal_pose_stamped.header.stamp = self.get_clock().now().to_msg()
            goal_pose_stamped.header.frame_id = self.robot_frame
            goal_pose_stamped.pose = goal_pose

            path.poses.append(start_pose)
            path.poses.append(goal_pose_stamped)

            self.path_publisher.publish(path)
            self.get_logger().info('Published path from base_link to goal_pose')

        except tf2_ros.LookupException as e:
            self.get_logger().error(f'Transform lookup failed: {e}')
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().error(f'Transform extrapolation failed: {e}')

    def set_in_motion_callback(self, request, response):
        self.in_motion = request.data
        response.success = True
        response.message = f"in_motion set to {self.in_motion}"
        return response

    def path_callback(self, msg):
        self.path = msg
        self.current_index = 0
        self.get_logger().info(f"Received new path with {len(msg.poses)} poses")
        self.current_path_world = []
        for p in self.path.poses:
                self.current_path_world.append([p.pose.position.x, p.pose.position.y])


    def path_follower_timer_callback(self):

        if not self.in_motion:
            twist_command_zero = Twist()
            twist_command_zero.linear.x = 0.0
            twist_command_zero.angular.z = 0.0
            self.twist_publisher.publish(twist_command_zero)
            return

        if not hasattr(self, "current_path_world"):
            self.get_logger().warn("Path Not Yet Planned")
            return

        if len(self.current_path_world) == 0:
            return

        try:
            transform = self.tf_buffer.lookup_transform("odom", self.robot_frame, rclpy.time.Time())

            # Extract the robot's position and orientation in the "map" frame
            self.x = transform.transform.translation.x
            self.y = transform.transform.translation.y
            self.robot_yaw = self.yaw_from_quaternion(transform.transform.rotation.x,
                                                transform.transform.rotation.y,
                                                transform.transform.rotation.z,
                                                transform.transform.rotation.w)

            self.robot_position = [self.x, self.y]
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            # Log a warning if the transform cannot be obtained
            self.get_logger().warn("Could not get transform from map to body: {}".format(ex))
            return

        linear_velocity, angular_velocity, self.pursuit_index = self.pure_pursuit(
            self.x,
            self.y,
            self.robot_yaw,
            self.current_path_world,
            self.pursuit_index,
            SPEED,
            LOOKAHEAD_DISTANCE
        )

        if(abs(self.x - self.current_path_world[-1][0]) < TARGET_ERROR and abs(self.y - self.current_path_world[-1][1]) < TARGET_ERROR):
            self.in_motion = False
            print("Target reached")
            linear_velocity = 0
            angular_velocity = 0
            self.current_path_world = []

        # Publish the twist commands
        twist_command = Twist()
        twist_command.linear.x = float(linear_velocity)
        twist_command.angular.z = float(angular_velocity)
        self.twist_publisher.publish(twist_command)


    def yaw_from_quaternion(self, x, y, z, w):
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def pure_pursuit(self, current_x, current_y, current_heading, path, index, speed, lookahead_distance, forward=True):
        closest_point = None
        for i in range(index, len(path)):
            x = path[i][0]
            y = path[i][1]
            distance = math.hypot(current_x - x, current_y - y)
            if lookahead_distance < distance:
                closest_point = (x, y)
                index = i
                break
        if closest_point is not None:


            # Calculate the lookahead point
            lookahead_angle = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)

            # Calculate the angle difference
            angle_diff = lookahead_angle - current_heading
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-pi, pi]

            # Decide the direction
            forward = abs(angle_diff) < math.pi / 2

            if forward:
                v = speed  # Set the speed to a negative value to make the robot go in reverse
            else:
                v = -speed  # Set the speed to a negative value to make the robot go in reverse

            if forward:
                target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)
            else:
                target_heading = math.atan2(current_y - closest_point[1], current_x - closest_point[0])  # Reverse the atan2 arguments
            desired_steering_angle = target_heading - current_heading
        else:
            if forward:
                target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)
            else:
                target_heading = math.atan2(current_y - path[-1][1], current_x - path[-1][0])  # Reverse the atan2 arguments
            desired_steering_angle = target_heading - current_heading
            index = len(path) - 1

             # Ensure v is assigned even if closest_point is None
            v = speed if forward else -speed

        if desired_steering_angle > math.pi:
            desired_steering_angle -= 2 * math.pi
        elif desired_steering_angle < -math.pi:
            desired_steering_angle += 2 * math.pi
        if desired_steering_angle > math.pi / 6 or desired_steering_angle < -math.pi / 6:
            sign = 1 if desired_steering_angle > 0 else -1
            desired_steering_angle = (sign * math.pi / 4)
            v = 0.0
        return v, desired_steering_angle, index

def main(args=None):
    rclpy.init()

    path_follower = SimplePurePursuit()

    rclpy.spin(path_follower)

    path_follower.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()