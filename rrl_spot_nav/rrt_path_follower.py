#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PointStamped, Pose, Twist
import numpy as np
import tf2_ros
from copy import deepcopy
from std_srvs.srv import SetBool

import heapq
import math
import scipy.interpolate as si
import numpy as np

EXPANSION_SIZE = 2
ROBOT_RADIUS = 0.3
SPEED = 0.4
LOOKAHEAD_DISTANCE = 0.4
TARGET_ERROR = 0.2
TARGET_ALLOWED_TIME = 10
MAP_TRIES = 10
FREE_SPACE_RADIUS = 5
UNEXPLORED_EDGES_SIZE = 6


class OpenCVFrontierDetector(Node):
    def __init__(self):
        super().__init__('detector')

        self.declare_parameter('path_topic', '/astar_planner_node/out/localPath')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('map_frame', 'map')

        self.path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value

        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.current_path_world = []
        self.path_subscriber = self.create_subscription(Path, self.path_topic, self.path_callback, 10)

        # Initialize the transform buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        #self.create_timer(0.5, self.frontier_timer_callback)
        self.create_timer(0.1, self.path_follower_timer_callback)

        self.in_motion = False
        self.pursuit_index = 0

        self.srv = self.create_service(SetBool, '/navigation/follow_path', self.set_in_motion_callback)

        self.target_allowed_time = TARGET_ALLOWED_TIME

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
                #pose = Pose()
                #pose.position.x, pose.position.y = map_to_world_coords(current_map, p[0], p[1])
                #path_marker.points.append(pose.position)
                self.current_path_world.append([p.pose.position.x, p.pose.position.y])

        # self.in_motion = True


    def path_follower_timer_callback(self):

        if not self.in_motion:
            return

        if not hasattr(self, "current_path_world"):
            self.get_logger().warn("Path Not Yet Planned")
            return

        if len(self.current_path_world) == 0:
            return

        try:
            transform = self.tf_buffer.lookup_transform("map", self.robot_frame, rclpy.time.Time())

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
        
        # if self.get_clock().now().to_msg().sec > self.target_allowed_time:
        #     self.in_motion = False
        #     print(f"Target not reached in {TARGET_ALLOWED_TIME} seconds, cancelling target...")
        #     linear_velocity = 0
        #     angular_velocity = 0

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
        if forward:
            v = speed  # Set the speed to a negative value to make the robot go in reverse
        else:
            v = -speed  # Set the speed to a negative value to make the robot go in reverse
        for i in range(index, len(path)):
            x = path[i][0]
            y = path[i][1]
            distance = math.hypot(current_x - x, current_y - y)
            if lookahead_distance < distance:
                closest_point = (x, y)
                index = i
                break
        if closest_point is not None:
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

    detector = OpenCVFrontierDetector()

    rclpy.spin(detector)

    detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()