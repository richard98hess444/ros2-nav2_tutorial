#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from math import radians, sin, cos

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        # Create a publisher on the /initialpose topic
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)

        # Get x, y, and theta from input parameters
        self.declare_parameter('x', 6.15)
        self.declare_parameter('y', 4.55)
        self.declare_parameter('theta', -90)

        # Publish the initial pose
        # self.publish_initial_pose()
        self.timer = self.create_timer(0.5, self.publish_initial_pose)

    def publish_initial_pose(self):
        # Create a PoseWithCovarianceStamped message
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
        initial_pose_msg.header.frame_id = 'map'  # Assuming map frame for localization

        # Get the x, y, theta parameters
        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        theta = radians(self.get_parameter('theta').value)  # Convert degrees to radians

        # Set position (x, y, z)
        initial_pose_msg.pose.pose.position.x = x
        initial_pose_msg.pose.pose.position.y = y
        initial_pose_msg.pose.pose.position.z = 0.0  # Assume 2D pose

        # Set orientation (quaternion) from theta
        initial_pose_msg.pose.pose.orientation.z = sin(theta / 2.0)
        initial_pose_msg.pose.pose.orientation.w = cos(theta / 2.0)

        # Set covariance (default to a small value for simplicity)
        initial_pose_msg.pose.covariance = [0.0] * 36
        initial_pose_msg.pose.covariance[0] = 0.1  # x covariance
        initial_pose_msg.pose.covariance[7] = 0.1  # y covariance
        initial_pose_msg.pose.covariance[35] = 0.1  # theta covariance

        # Publish the message
        self.publisher_.publish(initial_pose_msg)
        self.get_logger().info(f"Initial pose published: x={x}, y={y}, theta={theta} radians")


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
