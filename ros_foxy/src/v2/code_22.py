#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import StaticTransformBroadcaster
from math import sin, cos

class StaticTransformPublisher(Node):

    def __init__(self):
        super().__init__('static_transform_publisher')

        # Initialize TF2 Static Transform Broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # Define the transformation from base_link to lidar
        self.transform_base_to_lidar = TransformStamped()
        self.transform_base_to_lidar.header.stamp = self.get_clock().now().to_msg()
        self.transform_base_to_lidar.header.frame_id = 'base_link'
        self.transform_base_to_lidar.child_frame_id = 'chassis'

        # Set translation values (adjust as needed)
        self.transform_base_to_lidar.transform.translation.x = -0.09  # Adjust this value as needed
        self.transform_base_to_lidar.transform.translation.y = 0.0  # Adjust this value as needed
        self.transform_base_to_lidar.transform.translation.z = 0.112  # Adjust this value as needed

        # Set orientation as quaternion (adjust as needed)
        quaternion = Quaternion()
        quaternion.x = 0.0  # Adjust this value as needed
        quaternion.y = 0.0  # Adjust this value as needed
        quaternion.z = 0.0  # Adjust this value as needed
        quaternion.w = 1.0  # Adjust this value as needed
        self.transform_base_to_lidar.transform.rotation = quaternion

        # Publish the static transformation
        self.tf_broadcaster.sendTransform(self.transform_base_to_lidar)
        
        
        
        

def main(args=None):
    rclpy.init(args=args)
    static_transform_publisher = StaticTransformPublisher()
    rclpy.spin(static_transform_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
