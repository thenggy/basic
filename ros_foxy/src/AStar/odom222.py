#!/usr/bin/env python

import math
import time
import tf2_py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3Stamped, Pose2D, PoseWithCovarianceStamped
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
# from tf_transformations import quaternion_from_euler, euler_from_quaternion

def euler_to_quaternion(phi, theta, psi):
    """
    Convert Euler angles (phi, theta, psi) to a quaternion (w, x, y, z).
    
    Args:
    phi (float): Roll angle in radians.
    theta (float): Pitch angle in radians.
    psi (float): Yaw angle in radians.
    
    Returns:
    tuple: Quaternion (w, x, y, z).
    """
    cy = math.cos(psi * 0.5)
    sy = math.sin(psi * 0.5)
    cp = math.cos(theta * 0.5)
    sp = math.sin(theta * 0.5)
    cr = math.cos(phi * 0.5)
    sr = math.sin(phi * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return (qw, qx, qy, qz)
def quaternion_to_euler(x, y, z, w):
    # Roll (rotation around the X-axis)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
    
    # Pitch (rotation around the Y-axis)
    pitch = math.asin(2 * (w * y - z * x))
    
    # Yaw (rotation around the Z-axis)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    
    return roll, pitch, yaw

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        self.position_subscription = self.create_subscription(
            Vector3Stamped,
            'position',
            self.position_callback,
            1
        )
        
        self.imu_subscription = self.create_subscription(
            Imu,
            'imu/data_raw',
            self.imu_callback,
            1
        )
        
        self.yaw_publisher = self.create_publisher(
            Imu,
            'yaw',
            1
        )
        
        self.odom_publisher = self.create_publisher(
            Odometry,
            'odom',
            1
        )
        self.yaw_publisher_2 = self.create_publisher(
            Float64,
            'yaw2',
            1
        )

        self.localization_subscription = self.create_subscription(
            Pose2D,
            'localization',
            self.localization_callback,
            1
        )
        self.Pose_Estimate_subscription = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_estimate_callback,1)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.Ts = 0.01
        self.currentTime = time.perf_counter()
        self.prevTime = time.perf_counter() - 0.01
        
        self.old_x = 0.0
        self.old_y = 0.0
        self.old_theta = 0.0
        
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_position_set = False
        
        self.home_theta = 0.0
        self.home_orientation_set = False

        self.x_offset = 0.0
        self.y_offset = 0.0
        self.theta_offset = 0.0
        
        self.roll = 0.0
        self.yaw = 0.0 
        self.pitch = 0.0 
        self.estimate_odom_y = 0.0 
        self.estimate_odom_x = 0.0 

    def localization_callback(self, msg):
        self.x_offset += 0.5 * msg.x
        self.y_offset += 0.5 * msg.y
        self.theta_offset += 0.5 * msg.theta
    
    def imu_callback(self, msg):
        
        qw = msg.orientation.w
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        omega = msg.angular_velocity.z
        yaw_read = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)) 
        # self.get_logger().info('yaw_read = %f' % (yaw_read))
        (roll, pitch, yaw) = quaternion_to_euler(qx,qy,qz,qw)
        if self.home_orientation_set == False:
            self.home_theta = yaw
            self.roll = roll 
            self.yaw = yaw 
            self.pitch = pitch 
            self.home_orientation_set = True

        theta = yaw - self.home_theta
        self.get_logger().info('yaw_read = %f' % (theta))
        roll1 = roll - self.roll 
        pitch1 = pitch - self.pitch 
        yaw1 = yaw - self.yaw 
        
        if theta > math.pi:
            theta -= 2 * math.pi
        elif theta < -math.pi:
            theta += 2 * math.pi
            
        yaw_msg_1 = Float64()
        yaw_msg_1.data = theta
        
        self.yaw_publisher_2.publish(yaw_msg_1)
            
        yaw_msg = Imu()
        yaw_msg.header.stamp = self.get_clock().now().to_msg() 
        yaw_msg.header.frame_id = "bno055"
        qw1,qx1, qy1, qz1 = euler_to_quaternion(roll1, pitch1, yaw1)
        yaw_read = math.atan2(2.0 * (qw1 * qz1 + qx1 * qy1), 1.0 - 2.0 * (qy1 * qy1 + qz1 * qz1)) + self.theta_offset
        # self.get_logger().info('yaw_read = %f' % (yaw_read))
        yaw_msg.orientation.x = 0.0
        yaw_msg.orientation.y = 0.0
        yaw_msg.orientation.z = qz1
        yaw_msg.orientation.w = qw1
        yaw_msg.angular_velocity.x = 0.0
        yaw_msg.angular_velocity.y = 0.0
        yaw_msg.angular_velocity.z = omega
        yaw_msg.linear_acceleration.x = 0.0
        yaw_msg.linear_acceleration.y = 0.0
        yaw_msg.linear_acceleration.z = 0.0
       
        
        self.yaw_publisher.publish(yaw_msg)
        
    def pose_estimate_callback(self, msg):
        self.estimate_odom_x = msg.pose.pose.position.x
        self.estimate_odom_y = msg.pose.pose.position.y 
    
    def position_callback(self, msg):
        if self.home_position_set == False:
            self.home_x = msg.vector.x
            self.home_y = msg.vector.y
            self.home_position_set = True
            
        self.currentTime = time.perf_counter()
        Ts = self.currentTime - self.prevTime
        self.prevTime = self.currentTime

        self.x = msg.vector.x - self.home_x + self.x_offset 
        self.y = msg.vector.y - self.home_y + self.y_offset 
        self.theta = msg.vector.z
        
        self.vx = (self.x - self.old_x)/Ts
        self.vy = (self.y - self.old_y) /Ts
        self.old_x = self.x
        self.old_y = self.y
        
        self.wz = (self.theta - self.old_theta) / Ts
        self.old_theta = self.theta
        
        w,x,y,z= euler_to_quaternion(0, 0, self.theta)
        robot_orientation = [w,x,y,z]
        timestamp = self.get_clock().now().to_msg() 
        t = TransformStamped()

        t.header.stamp = timestamp 
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.w= w
        t.transform.rotation.z= z

        # odometry twist
        odom_msg = Odometry()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.header.stamp = self.get_clock().now().to_msg() 
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.w = w
        odom_msg.pose.pose.orientation.z = z
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.angular.z = self.wz
        self.tf_broadcaster.sendTransform(t)
        self.odom_publisher.publish(odom_msg)
        # self.get_logger().info('x = %f, y = : %f, theta = %f' % (self.x, self.y, self.theta))

def main(args=None):
    rclpy.init(args=args)
    robot_control_node = OdomPublisher()
    rclpy.spin(robot_control_node)

    robot_control_node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
