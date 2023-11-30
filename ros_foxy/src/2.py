#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import LaserScan
from math import atan2, hypot, sqrt, pi

def distance(x1, y1, x2, y2):
    return sqrt((x2 - x1)**2 + (y2 - y1)**2)

class P_Path(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.path_subscription = self.create_subscription(Path, '/robot_path', self.path_callback, 1)
        # self.lidar_subscription = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 1)
        self.p_controller = self.create_publisher(Twist, '/cmd_vel', 1)
        self.point_goal_publisher = self.create_publisher(PointStamped, '/current_goal', 1)

        self.path = Path()
        self.rx = [0.0]
        self.ry = [0.0]
        self.gx = 0.0
        self.gy = 0.0
        self.final_yaw = 0.0

        self.start = 0
        
        self.obstacle_detected = False
        
    # def lidar_callback(self, msg):
    #     print(msg.ranges[0])
    #     self.obstacle_detected = False
    #     for i in range (24):
    #         if msg.ranges[180 - i] < 1.2 or msg.ranges[180 + i] < 1.2:
    #             self.obstacle_detected = True
    #             break
        

    def odom_callback(self, msg):
        pose = msg.pose.pose
        
        self.sx = pose.position.x
        self.sy = pose.position.y

        qw = pose.orientation.w
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z

        self.yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

        if len(self.rx) > 2:
            for i in range(len(self.rx)):
                distance_to_trajectory = distance(self.sx, self.sy, self.rx[i], self.ry[i])
                
                print(distance_to_trajectory)
                if distance_to_trajectory < 0.8:
                    self.gx = self.rx[i]
                    self.gy = self.ry[i]
                    break  # Exit the loop after finding the next goal

        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = 'map'

        point_msg.point.x = self.gx
        point_msg.point.y = self.gy
        point_msg.point.z = 0.0
        self.point_goal_publisher.publish(point_msg)

        direction = [self.gx - self.sx, self.gy - self.sy]
        velocity = hypot(direction[1], direction[0])
        theta_difference = atan2(direction[1], direction[0]) - self.yaw

        if theta_difference > pi:
            theta_difference -= 2 * pi
        elif theta_difference < -pi:
            theta_difference += 2 * pi
        
        kp_v = 1.2
        kp_w = 1.2

        cmd_msg = Twist()
        
        if abs(theta_difference) > 0.5:
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = kp_w * theta_difference
        elif abs(theta_difference) < 0.5:
            cmd_msg.linear.x = kp_v * velocity
            cmd_msg.angular.z = kp_w * theta_difference

        if distance(self.sx, self.sy, self.rx[0], self.ry[0]) < 0.05:
            theta_difference = self.final_yaw - self.yaw

            if theta_difference > pi:
                theta_difference -= 2 * pi
            elif theta_difference < -pi:
                theta_difference += 2 * pi

            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = kp_w * theta_difference
            
        if self.obstacle_detected == True:
            cmd_msg.linear.x = 0.0


        if self.start:
            self.point_goal_publisher.publish(point_msg)
            self.p_controller.publish(cmd_msg)

    def path_callback(self, msg):
        self.path = msg
        self.rx = [msg.poses[i].pose.position.x for i in range(len(msg.poses))]
        self.ry = [msg.poses[i].pose.position.y for i in range(len(msg.poses))]
        self.final_orientation = msg.poses[0].pose.orientation
        qw = self.final_orientation.w
        qx = self.final_orientation.x
        qy = self.final_orientation.y
        qz = self.final_orientation.z
        self.final_yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

        if self.start == 0:
            self.start = 1
        

def main(args=None):
    rclpy.init(args=args)
    node = P_Path()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()