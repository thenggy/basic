#!/usr/bin/env python3

import rclpy

from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import ReliabilityPolicy, QoSProfile
from nav_msgs.msg import Path
from mte544_a_star.a_star_skeleton1 import find_path
# from sent_path import find_path
import numpy as np
# from mte544_action_interfaces.action import Move2Goal
from skimage.transform import downscale_local_mean # scale map
from math import atan2, hypot, sqrt, pi

import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PointStamped, Twist, PoseWithCovarianceStamped,PoseStamped
from sensor_msgs.msg import LaserScan


class AStarActionServer(Node):
    """ROS2 Action Server that plans path to a goal pose using A* and then follows the path using P-controller"""

    def __init__(self):
        super().__init__('a_star_action_server')
        # Setup Action server for receiving goal poses
        
        # create the subscriber object to Nav2 costmap
        self.global_map_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.map_callback, QoSProfile(depth=300, reliability=ReliabilityPolicy.BEST_EFFORT))
        # self.map_subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_pose_rviz_callback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        self.Pose_Estimate_subscription = self.create_subscription(PoseWithCovarianceStamped, '/abc', self.pose_estimate_callback,1)
        
        self.timer = self.create_timer(0.1, self.follow_path)

        # Create path visualizer publisher 
                # Create path visualizer publisher 
        self.path_pub = self.create_publisher(Path, '/path_viz', 10)

        # Initialize map parameters to default values. Will be correctly loaded later
        self.origin = [0,0,0]
        self.map_res = 0.05
        # Global costmap variable
        self.occupancy_map = None
        
        # List of cartisian points containing A* path to goal, in map frame in meters
        # self.path_cart = []
        
        self.goal_pose = None
        
        self.start=[]
        
        self.start_goal= False 
        self.reached_intermediate_goal = False
        
        self.new_goal_received = False
        self.b = False 
        # P-control publisher
        self.publisher_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        # The P-controller runs in the global frame.
        self.curr_pose = Pose() # holds current global position of turtlebot.
        self.setpoint_pose = Pose() # defaults to 0 till we receive a new setpoint from external node
        self.vel_msg = Twist() # holds velocity command to send to turtlebot
        self.reached_intermediate_goal = False

        # Used for finding TF between base_link frame and map (i.e. robot position)
        # See https://docs.ros.org/en/galactic/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html#write-the-listener-node
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.POSITION_THRESHOLD = 0.1
        self.path_cart = np.array([])
        
        self.laser_scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_scan_callback,
            10)
        self.current_laser_scan = None 

     
    def pose_estimate_callback(self, initial_pose):
        """Determine whether to accept or reject the goal"""
        # print(initial_pose)
        while True:
            # wait till global costmap has been received
            if self.occupancy_map is None:
                print("asdasds")
                pass
            else:
                print("steav mg")
                break
            
        
        self.start = self.coord_2_pixel((initial_pose.pose.pose.position.x, initial_pose.pose.pose.position.y))
        # print(initial_pose.pose.pose.position.x)
        self.curr_pose.x = initial_pose.pose.pose.position.x
        self.curr_pose.y = initial_pose.pose.pose.position.y
        if self.new_goal_received :
            self.get_path()
        # self.get_path()

    def update_occupancy_map_with_obstacles(self):
        if self.current_laser_scan is None:
            return

        for i, range in enumerate(self.current_laser_scan.ranges):
            if range < self.current_laser_scan.range_min or range > self.current_laser_scan.range_max:
                continue  # Ignore invalid ranges

            # Calculate the angle of the current laser scan point
            angle = self.current_laser_scan.angle_min + i * self.current_laser_scan.angle_increment

            # Calculate the coordinates in the robot frame
            x_robot = range * math.cos(angle)
            y_robot = range * math.sin(angle)

            # Transform these coordinates to the map frame
            # This requires knowing the current position and orientation of the robot in the map frame
            x_map, y_map = self.transform_to_map_frame(x_robot, y_robot)

            # Convert these map coordinates to occupancy grid cells
            cell_x, cell_y = self.coord_2_pixel((x_map, y_map))

            # Update the occupancy grid
            # Make sure to check the bounds of the grid
            if 0 <= cell_x < self.occupancy_map.shape[0] and 0 <= cell_y < self.occupancy_map.shape[1]:
                self.occupancy_map[cell_x, cell_y] = 100  # Mark as occupied (assuming 100 represents occupied space)

    def transform_to_map_frame(self, x_robot, y_robot):
        # Assuming self.curr_pose contains the current position (x, y) and orientation (theta) of the robot in the map frame
        theta = self.curr_pose.theta  # Robot's orientation in the map frame

        # Rotation (due to robot's orientation)
        x_map_rotated = x_robot * math.cos(theta) - y_robot * math.sin(theta)
        y_map_rotated = x_robot * math.sin(theta) + y_robot * math.cos(theta)

        # Translation (due to robot's position)
        x_map = x_map_rotated + self.curr_pose.x
        y_map = y_map_rotated + self.curr_pose.y

        return x_map, y_map

    def get_path(self):
        self.update_occupancy_map_with_obstacles()
        
        if not self.start or len(self.start) < 2:
            self.get_logger().info("Start position not ready.")
            return
        
        goal = self.coord_2_pixel((self.goal_pose.pose.position.x, self.goal_pose.pose.position.y))

        # Run A* and check if a valid path is found
        [path, cost] = find_path(self.start, goal, self.occupancy_map)
        self.estimated_heuristic = cost
        print(path.size )
        if path.size > 0:
            # Failed to find a path to the goal
            # self.get_logger().info("No path found")
            
            path_scale = path * self.map_res
            self.path_cart = path_scale + self.origin
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            for pose in self.path_cart:
                point = PoseStamped()
                point.pose.position.x = pose[0]
                point.pose.position.y = pose[1]
                path_msg.poses.append(point)
            self.path_pub.publish(path_msg)

            self.get_logger().info(f"Number of points: {self.path_cart.shape[0]}")
            # self.visualize_path()
            self.start_goal = True
        else:
            self.get_logger().info("No path found")
            self.path_cart = np.array([])
            self.start_goal = False

    def visualize_path(self):
    
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for pose in self.path_cart:
            point = PoseStamped()
            point.pose.position.x = pose[0]
            point.pose.position.y = pose[1]
            path_msg.poses.append(point)
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Path visualized with {len(self.path_cart)} points")
       
    def follow_path(self):
        if not self.start_goal or len(self.path_cart) == 0:
            return  # No path to follow or goal not set

        # Check if there is a new goal
        if self.new_goal_received:
            self.handle_new_goal()
            return  # Start following the new path

        # Move towards the current point on the path
        current_point = self.path_cart[0]
        self.setpoint_pose.x = current_point[0]
        self.setpoint_pose.y = current_point[1]

        # Check if the intermediate goal is reached
        if self.get_position_error() < self.POSITION_THRESHOLD:
            if len(self.path_cart) > 1:
                self.path_cart = self.path_cart[1:, :]  

            else:
                self.start_goal = False
                self.reached_intermediate_goal = True
                self.stop_robot()
                return
        self.reached_intermediate_goal = False
        self.update_current_pose()
        self.run_control_loop_once()
    # Continue in the next call



    def handle_new_goal(self):
        """Handle the process when a new goal is received."""
        self.new_goal_received = False
        self.start = self.coord_2_pixel((self.curr_pose.x, self.curr_pose.y))
        self.get_path()  # Recalculate the path for the new goal

                
    def stop_robot(self):
        # Code to stop the robot
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0
        self.publisher_vel.publish(self.vel_msg)
        print("Robot stopped at final goal")        

    def map_callback(self, msg):
        """Load received global costmap"""

        if self.occupancy_map is None:
            self.origin = [msg.info.origin.position.x, msg.info.origin.position.y]
            self.height = msg.info.height
            self.width = msg.info.width
            self.map_res = msg.info.resolution
            self.occupancy_map = np.reshape(msg.data, (self.height, self.width))
            
            # downsample map for faster processing
            self.occupancy_map = downscale_local_mean(self.occupancy_map, (2,2))
            self.map_res *= 2
            
            # Convert costmap message from y-by-x to x-by-y coordinates as it is row-major order, with (0,0) at bottom left
            self.occupancy_map = np.transpose(self.occupancy_map)

    def coord_2_pixel(self, point):
        """Convert a coordinate in map frame (meters) to pixels"""
        return (
            round((point[0] - self.origin[0])/(self.map_res)), 
            round((point[1] - self.origin[1])/(self.map_res))
        ) 

   
        #P-controller helper functions
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
            quaternion = (
                base_map_rot.x,
                base_map_rot.y,
                base_map_rot.z,
                base_map_rot.w
                )
            _, _, self.curr_pose.theta = self.euler_from_quaternion(quaternion)

        except TransformException as ex:
            print(f"TransformException: {ex}")
            return
    def euler_from_quaternion(self, quaternion):       
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        (x,y,z,w) = quaternion
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
    
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
    
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
    
        return roll_x, pitch_y, yaw_z # in radians
    
    def bound(self, val:float, bound:float) ->float:
        """Limit value to given symmetric range"""
        return min(max(val, -bound), bound)
    
    def get_position_error(self):
        """Calculate error in position between current pose and the setpoint pose"""
        # As we independantly have another P controller to turn towards the setpoint, we can 
        # think of the position error as a straight line away from our current position.
        # So, use Euclidean distance as the error function.

        return math.sqrt(math.pow((self.setpoint_pose.x - self.curr_pose.x), 2) +
                         math.pow((self.setpoint_pose.y - self.curr_pose.y), 2))
        
        
    def get_linear_velocity(self, Kp=1.4) -> float:
        """Proportional controller for Position"""
        # if we need to rotate more than 60 degrees (arbitrarily), we should first turn and then move forward later
        # This will allow robot to stay on course
        if abs(self.get_angle_error()) > math.pi/4.0:
            return 0.0
        # Calculate proportional linear velocity
        return self.bound(Kp * self.get_position_error(), 0.2) # Max linear velocity from Turtlebot4 datasheet: 0.3m/s

    def get_required_theta(self):
        """Calculate angle needed for turtlebot to face the setpoint"""
        return math.atan2(self.setpoint_pose.y - self.curr_pose.y, self.setpoint_pose.x - self.curr_pose.x)

    def get_angle_error(self):
        '''Calculate change in angle needed for turtlebot to face the setpoint'''
        angle_error = self.get_required_theta() - self.curr_pose.theta
        # if current theta is 3.14 and desired theta is -3.1415, the smallest rotation angle would be 0.0015 rad 'left' 
        # and the rotation shouldn't be 1 whole round rotation : = -6.2 rad
        # So, Accounts for discontinous jump at pi -> -pi
        # by adding or subtracting a full turn (2pi rad) when the angle resulting angle is outside the (-pi, pi) range.
        angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi
        return angle_error

    def get_angular_velocity(self, Kp=2.5) -> float:
        '''Proportional controller for orientation to face towards setpoint position'''
        # Calculate proprotional angular velocity
        angular_velocity = 0.0
        
        # self.bound(Kp * self.get_angle_error(), 90.0) # Max angular velocity from Turtlebot4 datasheet: 1.9rad/s
        angular_velocity = Kp * self.get_angle_error()

        angular_velocity = max(-0.5,min(angular_velocity, 0.5))

        return angular_velocity

        
    def run_control_loop_once(self):
        SMOOTHING_FACTOR = 0.5  # Start with a lower value, like 0.05

        # Calculate desired velocities from P-controller
        desired_linear_vel = self.get_linear_velocity()
        desired_angular_vel = self.get_angular_velocity()

        # Apply smoothing
        self.vel_msg.linear.x += (desired_linear_vel - self.vel_msg.linear.x) * SMOOTHING_FACTOR
        self.vel_msg.angular.z += (desired_angular_vel - self.vel_msg.angular.z) * SMOOTHING_FACTOR

        # Ensure velocities are within bounds
        self.vel_msg.linear.x = self.bound(self.vel_msg.linear.x, 0.2)  # Linear velocity bound
        self.vel_msg.angular.z = max(-0.5, min(self.vel_msg.angular.z, 0.5))  # Angular velocity bound
        self.get_logger().info(f'Linear Velocity: {self.vel_msg.linear.x}, Angular Velocity: {self.vel_msg.angular.z}')

        # Publish the velocity command
        self.publisher_vel.publish(self.vel_msg)
        
    # def run_control_loop_once(self):
    #     if not self.start_goal or len(self.path_cart) == 0:
    #         return  # No path to follow or goal not set

    #     # Current target point
    #     current_point = self.path_cart[0]
    #     direction = [current_point[0] - self.curr_pose.x, current_point[1] - self.curr_pose.y]
    #     velocity = hypot(direction[0], direction[1])
    #     theta_goal = atan2(direction[1], direction[0])

    #     # Calculate angle difference
    #     theta_difference = theta_goal - self.curr_pose.theta
    #     if theta_difference > pi:
    #         theta_difference -= 2 * pi
    #     elif theta_difference < -pi:
    #         theta_difference += 2 * pi

    #     # Proportional controller gains
    #     kp_v = 0.5  # Linear velocity gain
    #     kp_w = 1.2  # Angular velocity gain

    #     # Calculate linear and angular velocities
    #     if abs(theta_difference) > 0.5:
    #         self.vel_msg.linear.x = 0.0
    #         self.vel_msg.angular.z = kp_w * theta_difference
    #     else:
    #         self.vel_msg.linear.x = kp_v * velocity
    #         self.vel_msg.angular.z = kp_w * theta_difference

    #     # Publish the velocity command
    #     self.publisher_vel.publish(self.vel_msg)

    #     # Check if the robot has reached the current target point
    #     if hypot(self.curr_pose.x - current_point[0], self.curr_pose.y - current_point[1]) < 0.1:
    #         # Move to the next point in the path
    #         self.path_cart.pop(0)


    def detect_obstacle(self, laser_scan_msg):
        FRONT_OBSTACLE_DISTANCE_THRESHOLD = 0.1  # 0.25 meters, slightly more than robot's length
        FRONT_ARC_DEGREES = 4  # Narrow frontal arc (in degrees)

        num_ranges = len(laser_scan_msg.ranges)
        middle_idx = num_ranges // 2
        arc_half_width = int((FRONT_ARC_DEGREES / 360) * num_ranges)

        front_start_idx = max(middle_idx - arc_half_width, 0)
        front_end_idx = min(middle_idx + arc_half_width, num_ranges)
        
        
        for i in range(front_start_idx, front_end_idx):
            if 0 < laser_scan_msg.ranges[i] < FRONT_OBSTACLE_DISTANCE_THRESHOLD:
                print(front_start_idx)
                # self.get_logger().info(f"Obstacle detected at index {i}, range: {laser_scan_msg.ranges[i]}")
                return True  # Obstacle detected within threshold in the frontal arc

        return False

    def laser_scan_callback(self, msg):
        # Update the current laser scan data
        self.current_laser_scan = msg
    
    def goal_pose_rviz_callback(self, msg):
        """Store `2D Goal Pose` from RViz"""
        self.goal_pose = msg
        self.get_logger().info(f"Goal position: {self.goal_pose.pose.position.x, self.goal_pose.pose.position.y}")
        self.new_goal_received = True
        self.start_goal= True 
    

def main(args=None):
    rclpy.init(args=args)
    a_star_action_server = AStarActionServer()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(a_star_action_server)
            # follow_path logic is now handled within get_path
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()