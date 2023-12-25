#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
from scipy.interpolate import interp1d
from scipy.signal import savgol_filter
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped

import math
from sensor_msgs.msg import LaserScan

show_animation = True

import matplotlib.pyplot as plt

from turtlesim.msg import Pose


from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from rclpy.qos import ReliabilityPolicy, QoSProfile

import numpy as np
from skimage.transform import downscale_local_mean


def smooth_trajectory(trajectory_x, trajectory_y, window_size=20, poly_order=2):
    smoothed_x = savgol_filter(trajectory_x, window_size, poly_order)
    smoothed_y = savgol_filter(trajectory_y, window_size, poly_order)
    return smoothed_x, smoothed_y





class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

       
       
       
        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

        
        
        
            # show graph
            show_animation = 0
            
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                #print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    
    
    
    
    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    
    
    
    
    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

   
   
   
   
    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion

def remove_close_obstacles(obstacle_x_list, obstacle_y_list, min_distance=0.2):
    new_obstacle_x = []
    new_obstacle_y = []

    for i in range(len(obstacle_x_list)):
        is_far_enough = True

        for j in range(len(new_obstacle_x)):
            distance = ((obstacle_x_list[i] - new_obstacle_x[j])**2 +
                        (obstacle_y_list[i] - new_obstacle_y[j])**2)**0.5

            if distance < min_distance:
                is_far_enough = False
                break

        if is_far_enough:
            new_obstacle_x.append(obstacle_x_list[i])
            new_obstacle_y.append(obstacle_y_list[i])

    return new_obstacle_x, new_obstacle_y

class PathPublisherNode(Node):
    def __init__(self):
        super().__init__('path_publisher')
        # self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.goal_subscription = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 1)
        # self.map_subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)
        self.tf_subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            1)
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            1)
        self.path_publisher = self.create_publisher(Path, '/robot_path', 1)
        self.Pose_Estimate_publish = self.create_publisher(PoseWithCovarianceStamped, '/abc',1)
        self.global_map_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.map_callback, QoSProfile(depth=300, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.path = Path()

        # self.sx = 0.0
        # self.sy = 0.0
        self.gx = 0.0
        self.gy = 0.0
        
        
        
        
        
        self.ox = [0.0]
        self.oy = [0.0]

        # self.yaw = 0.0
        
        self.map_to_odom_x = 0.0
        self.map_to_odom_y = 0.0
        
        self.map_ox = []
        self.map_oy = []
        self.laser_ox = []
        self.laser_oy = []
        # self.ox = np.concatenate((self.map_ox, self.laser_ox))
        # self.oy = np.concatenate((self.map_oy, self.laser_oy))
        self.rx = np.array([])
        self.ry = np.array([])

        # self.yaw = 0.0

        self.grid_size = 0.2 # [m]
        self.robot_radius = 0.5 # [m]


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.a_star = AStarPlanner(self.ox, self.oy, self.grid_size, self.robot_radius)
        
        self.goal_pose = None
        
        self.curr_pose = Pose()
        
        self.occupancy_map  = None
        self.map_res = 0.05
        self.origin = [0,0,0]
    
    def laser_callback(self, msg):
        # print("Laser received!")
        max_range = 1.0

        valid_ranges = (0.0 < np.array(msg.ranges)) & (np.array(msg.ranges) < max_range)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        x = msg.ranges * np.cos(angles)
        y = msg.ranges * np.sin(angles)
            
        # Rotate the points based on the robot's current yaw
        rotated_x = x * np.cos(self.curr_pose.theta) - y * np.sin(self.curr_pose.theta)
        rotated_y = x * np.sin(self.curr_pose.theta) + y * np.cos(self.curr_pose.theta)
        
        self.laser_ox, self.laser_oy = remove_close_obstacles(rotated_x[valid_ranges] + self.curr_pose.x, rotated_y[valid_ranges] + self.curr_pose.y, 0.1)
        

        # self.ox =  self.laser_ox
        # self.oy = self.laser_oy
        if not self.ox or not self.oy:
        # If ox and oy are empty, initialize them with some default values or skip calculation
            return
        
        self.ox, self.oy = remove_close_obstacles(self.ox, self.oy, 0.1)
        self.a_star.calc_obstacle_map(self.ox, self.oy)

        # distances = np.sqrt((self.rx[:, np.newaxis] - self.laser_ox)**2 + (self.ry[:, np.newaxis] - self.laser_oy)**2)
        # obstacle_msg = Bool()
        plt.cla()
        show_animation = 1
        if show_animation:  # pragma: no cover
            plt.plot(self.ox, self.oy, ".k")
            plt.plot(self.curr_pose.x, self.curr_pose.y, "og")
            plt.plot(self.gx, self.gy, "xb")
            plt.grid(True)
            plt.axis("equal")
            plt.plot(self.rx, self.ry, "-r")
            plt.pause(0.001)
            #plt.show()
            plt.draw()

        # if np.any(distances <= 0.2):
        #     obstacle_msg.data = True
        #     if self.last_time_obstacle_detected is None:
        #         self.last_time_obstacle_detected = time.perf_counter()
        # else:
        #     obstacle_msg.data = False
        #     self.last_time_obstacle_detected = None

        # if self.last_time_obstacle_detected and time.perf_counter() - self.last_time_obstacle_detected >= 5 or len(self.rx) < 2:
        #     self.get_path()
        #     self.last_time_obstacle_detected = None 
        
        # self.obstacle_publisher.publish(obstacle_msg)

            
            
    def get_current_pose(self):
        """Get current pose of the robot as a PoseWithCovarianceStamped message"""
        from_frame_rel = 'base_link'
        to_frame_rel = 'map'
        try:
            t = self.tf_buffer.lookup_transform(to_frame_rel, from_frame_rel, rclpy.time.Time())
            self.curr_pose.x = t.transform.translation.x
            self.curr_pose.y = t.transform.translation.y
            base_map_rot = t.transform.rotation

            # Convert quaternion to euler angles for internal yaw representation
            _, _, yaw = self.euler_from_quaternion(base_map_rot.x, base_map_rot.y, base_map_rot.z, base_map_rot.w)
            self.curr_pose.theta = yaw

            # Create PoseWithCovarianceStamped message
            initial_pose = PoseWithCovarianceStamped()
            initial_pose.pose.pose.position.x = self.curr_pose.x
            initial_pose.pose.pose.position.y = self.curr_pose.y
            initial_pose.pose.pose.orientation.x = base_map_rot.x
            initial_pose.pose.pose.orientation.y = base_map_rot.y
            initial_pose.pose.pose.orientation.z = base_map_rot.z
            initial_pose.pose.pose.orientation.w = base_map_rot.w
            print(self.curr_pose.x)
            print(self.curr_pose.y)
            print(self.curr_pose.theta)

            return initial_pose
        except TransformException as ex:
            self.get_logger().error(f"Failed to get current pose: {ex}")
            return None

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

    def send_goal(self):
        """Process the new goal pose."""
        goal_msg = PoseWithCovarianceStamped()
        try:
            if self.goal_pose is None:
                self.get_logger().info("No goal pose received yet.")
                return

            # Get the current pose of the Turtlebot
            self.initial_pose = self.get_current_pose()
            goal_msg = self.initial_pose

            if self.initial_pose is None:
                self.get_logger().info("Failed to get initial pose.")
                return

            # Log the initial pose
            self.get_logger().info(f"Initial Pose: {self.initial_pose.pose.pose.position.x}, {self.initial_pose.pose.pose.position.y}")

            # Add your logic here to handle the new goal pose.
            # For example, you can use both the initial pose and the goal pose for path planning,
            # sending commands to the robot, etc.
        
        except Exception as e:
            self.get_logger().error(f"Error in send_goal: {e}")    
            
        self.Pose_Estimate_publish.publish(goal_msg)

    def goal_pose_callback(self, msg):
        self.goal_pose = msg
        self.gx = self.goal_pose.pose.position.x + self.map_to_odom_x
        self.gy = self.goal_pose.pose.position.y + self.map_to_odom_y
        self.orientation = msg.pose.orientation
        self.send_goal()
        self.get_path()
    
    def get_path(self):
        
        self.rx, self.ry = self.a_star.planning(self.curr_pose.x, self.curr_pose.y, self.gx, self.gy)
        plt.cla()
        
        show_animation = 1
        if show_animation:  # pragma: no cover
            plt.plot(self.ox, self.oy, ".k")
            plt.plot(self.curr_pose.x, self.curr_pose.y, "og")
            plt.plot(self.gx, self.gy, "xb")
            plt.grid(True)
            plt.axis("equal")
            plt.plot(self.rx, self.ry, "-r")
            plt.pause(0.001)
            #plt.show()
            plt.draw()

        #print(np.array(self.rx).shape)

        # Create an interpolating function for x and y coordinates
        interpolator_x = interp1d(range(len(self.rx)), self.rx, kind='linear')
        interpolator_y = interp1d(range(len(self.ry)), self.ry, kind='linear')

        # Calculate the total length of the initial trajectory
        initial_length = np.sum(np.sqrt(np.diff(self.rx)**2 + np.diff(self.ry)**2))

        # Determine the number of segments needed
        num_segments = int(np.ceil(initial_length / 0.05))

        # Generate equidistant points along the trajectory using interpolation
        t_new = np.linspace(0, len(self.rx) - 1, num_segments + 1)
        rx_new = interpolator_x(t_new)
        ry_new = interpolator_y(t_new)

        self.rx = rx_new
        self.ry = ry_new

        try:
            self.rx, self.ry = smooth_trajectory(self.rx, self.ry)
        except:
            print("No obstacle")

        # Combine x and y coordinates to form the complete path
        complete_path = list(zip(rx_new, ry_new))

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        start = 0
        for x,y in zip(self.rx, self.ry):
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            if start == 0:
                pose_stamped.pose.orientation = self.orientation
                start = 1
            path_msg.poses.append(pose_stamped)

        #print(self.ry)

        self.path_publisher.publish(path_msg)   

    # def map_callback(self, msg):
    #     self.width = msg.info.width
    #     self.height = msg.info.height
    #     self.resolution = msg.info.resolution
    #     self.origin = msg.info.origin

    #     self.map_ox = []
    #     self.map_oy = []

    #     for y in range(self.height):
    #         for x in range(self.width):
    #             index = y * self.width + x
    #             cell_value = msg.data[index]

    #             if cell_value >= 40:
    #                 obstacle_x = self.origin.position.x + (x + 0.5) * self.resolution + self.map_to_odom_x # convert to meter
    #                 obstacle_y = self.origin.position.y + (y + 0.5) * self.resolution + self.map_to_odom_y
    #                 self.map_ox.append(obstacle_x)
    #                 self.map_oy.append(obstacle_y)

    #     self.ox, self.oy = remove_close_obstacles(self.map_ox, self.map_oy, 0.1)
        
    def map_callback(self, msg):
        """Load received global costmap and convert to obstacle lists"""
        # Code to merged
        if self.occupancy_map is None:
            self.origin = [msg.info.origin.position.x, msg.info.origin.position.y]
            self.height = msg.info.height
            self.width = msg.info.width
            self.map_res = msg.info.resolution
            self.occupancy_map = np.reshape(msg.data, (self.height, self.width))

            # Further downsample the map for faster processing
            downsample_factor = 6  # Increase this factor as needed
            self.occupancy_map = downscale_local_mean(self.occupancy_map, (downsample_factor, downsample_factor))
            self.map_res *= downsample_factor

            # Convert costmap message from y-by-x to x-by-y coordinates
            self.occupancy_map = np.transpose(self.occupancy_map)
            print("Downsampled map shape:", self.occupancy_map.shape)
            print("Sample values from downsampled map:", self.occupancy_map[0:10, 0:10])

            # Convert the occupancy map to obstacle coordinates
            self.ox, self.oy = [], []
            obstacle_threshold = 40  # Adjust as needed
            for ix in range(self.occupancy_map.shape[0]):
                for iy in range(self.occupancy_map.shape[1]):
                    if self.occupancy_map[ix, iy] > obstacle_threshold:
                        x_world = ix * self.map_res + self.origin[0]
                        y_world = iy * self.map_res + self.origin[1]
                        self.ox.append(x_world)
                        self.oy.append(y_world)
            # Check if ox and oy are populated
        if not self.ox or not self.oy:
            print("No obstacles detected after processing map.")
        else:
            print("Obstacles detected: ", len(self.ox))


    def tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                self.map_to_odom_x = transform.transform.translation.x
                self.map_to_odom_y = transform.transform.translation.y
                # print(self.map_to_odom_x, self.map_to_odom_y)


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
