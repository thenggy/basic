#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import LaserScan
from math import atan2, hypot, sqrt, pi
from geometry_msgs.msg import PoseWithCovarianceStamped

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
import math
from turtlesim.msg import Pose
def distance(x1, y1, x2, y2):
    return sqrt((x2 - x1)**2 + (y2 - y1)**2)

class P_Path(Node):
    # update again
    def __init__(self):
        super().__init__('path_publisher')
        # self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.path_subscription = self.create_subscription(Path, '/robot_path', self.path_callback, 1)
        # self.lidar_subscription = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 1)
        self.p_controller = self.create_publisher(Twist, '/cmd_vel', 1)
        self.point_goal_publisher = self.create_publisher(PointStamped, '/current_goal', 1)
        
        self.Pose_Estimate_subscription = self.create_subscription(PoseWithCovarianceStamped, '/abc', self.pose_estimate_callback,1)
        self.timer = self.create_timer(0.01, self.follow_path)


        self.path = Path()
        self.rx = [0.0]
        self.ry = [0.0]
        self.gx = 0.0
        self.gy = 0.0
        self.final_yaw = 0.0
        
        # self.sx = 0.0 
        # self.sy=0.0
        # self.yaw = 0.0

        self.start = 0
        
        self.obstacle_detected = False
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.curr_pose = Pose()
        
    # def lidar_callback(self, msg):
    #     print(msg.ranges[0])
    #     self.obstacle_detected = False
    #     for i in range (24):
    #         if msg.ranges[180 - i] < 1.2 or msg.ranges[180 + i] < 1.2:
    #             self.obstacle_detected = True
    #             break
    def pose_estimate_callback(self, initial_pose):
        """Determine whether to accept or reject the goal"""
        
        
        self.curr_pose.x = initial_pose.pose.pose.position.x
        self.curr_pose.y = initial_pose.pose.pose.position.y
        # print(initial_pose)
        
        # self.start = self.coord_2_pixel((initial_pose.pose.pose.position.x, initial_pose.pose.pose.position.y))
        # print(initial_pose.pose.pose.position.x)
        # self.sx = initial_pose.pose.pose.position.x
        # self.sy = initial_pose.pose.pose.position.y
        
        qw = initial_pose.pose.pose.orientation.w
        qx = initial_pose.pose.pose.orientation.x
        qy = initial_pose.pose.pose.orientation.y
        qz = initial_pose.pose.pose.orientation.z

        self.curr_pose.theta = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        
    def update_current_pose(self):
        """Get current pose of Turtlebot"""
        # Find the transform between base_link and map
        from_frame_rel = 'base_link'
        to_frame_rel = 'map'
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            self.curr_pose.x = t.transform.translation.x
            self.curr_pose.y = t.transform.translation.y
            base_map_rot = t.transform.rotation

            # Convert quaternion to euler angles for internal yaw representation
            _, _, yaw = self.euler_from_quaternion(base_map_rot.x, base_map_rot.y, base_map_rot.z, base_map_rot.w)
            self.curr_pose.theta = yaw
            # _, _, self.curr_pose.theta = self.euler_from_quaternion(quaternion)
            
            print(self.curr_pose.theta)
            # print(self.curr_pose.y)

        except TransformException as ex:
            print(f"TransformException: {ex}")
            return
        
    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        # roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp) # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    

    def follow_path(self):
        
        
        
        self.curr_pose.x = self.curr_pose.x
        self.curr_pose.y = self.curr_pose.y


        self.curr_pose.theta = self.curr_pose.theta

        if len(self.rx) > 2:
            for i in range(len(self.rx)):
                distance_to_trajectory = distance(self.curr_pose.x, self.curr_pose.y, self.rx[i], self.ry[i])
                
                if distance_to_trajectory < 0.8:
                    self.gx = self.rx[i]
                    self.gy = self.ry[i]
                    break  # Exit the loop after finding the next goal
        self.update_current_pose()

        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = 'map'

        point_msg.point.x = self.gx
        point_msg.point.y = self.gy
        point_msg.point.z = 0.0
        self.point_goal_publisher.publish(point_msg)

        direction = [self.gx - self.curr_pose.x, self.gy - self.curr_pose.y]
        velocity = hypot(direction[1], direction[0])
        theta_difference = atan2(direction[1], direction[0]) - self.curr_pose.theta

        if theta_difference > pi:
            theta_difference -= 2 * pi
        elif theta_difference < -pi:
            theta_difference += 2 * pi
        
        kp_v = 0.5
        kp_w = 1.2

        cmd_msg = Twist()
        
        if abs(theta_difference) > 0.5:
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = kp_w * theta_difference
        elif abs(theta_difference) < 0.5:
            cmd_msg.linear.x = kp_v * velocity
            cmd_msg.angular.z = kp_w * theta_difference

        if distance(self.curr_pose.x, self.curr_pose.y, self.rx[0], self.ry[0]) < 0.05:
            theta_difference = self.final_yaw - self.curr_pose.theta

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


    # def odom_callback(self, msg):
    #     pose = msg.pose.pose
        
    #     self.sx = pose.position.x
    #     self.sy = pose.position.y

    #     qw = pose.orientation.w
    #     qx = pose.orientation.x
    #     qy = pose.orientation.y
    #     qz = pose.orientation.z

    #     self.yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    #     if len(self.rx) > 2:
    #         for i in range(len(self.rx)):
    #             distance_to_trajectory = distance(self.sx, self.sy, self.rx[i], self.ry[i])
                
    #             if distance_to_trajectory < 0.8:
    #                 self.gx = self.rx[i]
    #                 self.gy = self.ry[i]
    #                 break  # Exit the loop after finding the next goal

    #     point_msg = PointStamped()
    #     point_msg.header.stamp = self.get_clock().now().to_msg()
    #     point_msg.header.frame_id = 'map'

    #     point_msg.point.x = self.gx
    #     point_msg.point.y = self.gy
    #     point_msg.point.z = 0.0
    #     self.point_goal_publisher.publish(point_msg)

    #     direction = [self.gx - self.sx, self.gy - self.sy]
    #     velocity = hypot(direction[1], direction[0])
    #     theta_difference = atan2(direction[1], direction[0]) - self.yaw

    #     if theta_difference > pi:
    #         theta_difference -= 2 * pi
    #     elif theta_difference < -pi:
    #         theta_difference += 2 * pi
        
    #     kp_v = 0.5
    #     kp_w = 1.2

    #     cmd_msg = Twist()
        
    #     if abs(theta_difference) > 0.5:
    #         cmd_msg.linear.x = 0.0
    #         cmd_msg.angular.z = kp_w * theta_difference
    #     elif abs(theta_difference) < 0.5:
    #         cmd_msg.linear.x = kp_v * velocity
    #         cmd_msg.angular.z = kp_w * theta_difference

    #     if distance(self.sx, self.sy, self.rx[0], self.ry[0]) < 0.05:
    #         theta_difference = self.final_yaw - self.yaw

    #         if theta_difference > pi:
    #             theta_difference -= 2 * pi
    #         elif theta_difference < -pi:
    #             theta_difference += 2 * pi

    #         cmd_msg.linear.x = 0.0
    #         cmd_msg.angular.z = kp_w * theta_difference
            
    #     if self.obstacle_detected == True:
    #         cmd_msg.linear.x = 0.0


    #     if self.start:
    #         self.point_goal_publisher.publish(point_msg)
    #         self.p_controller.publish(cmd_msg)

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