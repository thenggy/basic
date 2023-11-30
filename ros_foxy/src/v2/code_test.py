#!/usr/bin/env python

import math
from math import sin, cos, pi
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from tf2_ros import TransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Twist, Quaternion, TransformStamped,Vector3Stamped
from std_msgs.msg import Float64


def quaternion_from_euler(roll, pitch, yaw) -> Quaternion:
    cy = math.cos(yaw*0.5)
    sy = math.sin(yaw*0.5)
    cp = math.cos(pitch*0.5)
    sp = math.sin(pitch*0.5)
    cr = math.cos(roll*0.5)
    sr = math.sin(roll*0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr

    return q
class SerialStatus:
    """Class for different data given by the embedded system"""
    left_ref_speed: float
    right_ref_speed: float
    left_speed:float
    right_speed: float
    left_effort: float
    right_effor: float
    x_pos: float
    y_pos: float
    theta: float
    v: float
    w: float
    current_time: Float64
    delta_x:float
    delta_y:float
    delta_th:float
    x:float
    y:float
    th:float

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        self.subscription = self.create_subscription(
            Vector3Stamped,
            'speed_msg',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # self.last_time = self.get_clock().now().to_msg()
            # set timer
        # self.pub_period = 0.01  # 0.02 seconds = 50 hz = pid rate for robot
        # self.pub_timer = self.create_timer(self.pub_period, self.pub_callback)
        # tf
        self.odom_publisher = self.create_publisher(
            Odometry,
            'odom',
            10
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        
        
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.Ts = 0.01
        self.currentTime = 0.0
        self.prevTime = 0.0

        # current_time = self.get_clock().now().to_msg()
        # last_time = self.get_clock().now().to_msg()

        # self.rate = self.create_rate(1)
    def listener_callback(self, speed_msg):
        # vx= Vector3Stamped()
        
        self.currentTime = time.perf_counter()
        Ts = self.currentTime - self.prevTime
        self.prevTime = self.currentTime
        
        vx =speed_msg.vector.x
        vth = speed_msg.vector.z

        # delta_x = vx * cos(self.th)
        # delta_y = vx * sin(self.th)
        # delta_th = vth

        # self.x += delta_x * self.Ts
        # self.y += delta_y * self.Ts
        # self.th += delta_th* self.Ts 
        
        self.x += vx * cos(vth) * Ts
        self.y += vx * sin(vth)*Ts 
        self.th += vth * Ts *0.83
        
        print(vx)
        
        robot_orientation = quaternion_from_euler(0, 0, self.th)
        timestamp = self.get_clock().now().to_msg() 
        t = TransformStamped()
        t.header.stamp = timestamp 
        t.header.frame_id = '/odom'
        t.child_frame_id = '/base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = robot_orientation
        
        
        # odometry twist
        odom_msg = Odometry()
        odom_msg.header.frame_id = '/odom'
        odom_msg.child_frame_id = '/base_link'
        odom_msg.header.stamp = timestamp
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = robot_orientation
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.angular.z = vth
        self.tf_broadcaster.sendTransform(t)
        self.odom_publisher.publish(odom_msg)
        self.get_logger().info('Received: %f, Calculated: %f' % (self.x,self.th))
# 
        # first, we'll publish the transform over tf
def main(args=None):


    rclpy.init(args=args)
    robot_control_node = OdomPublisher()
    rclpy.spin(robot_control_node)

    robot_control_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()