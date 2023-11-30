#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage

import math

show_animation = True

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

def convert_to_obstacle_coordinates(input_path):
    try:
        # Open the image
        img = Image.open(input_path)

        # Convert to grayscale if not already
        img = img.convert("L")
        
        # Get the image size
        width, height = img.size

        # List to store obstacle coordinates
        obstacle_coordinates = []

        # Iterate through pixels and find obstacle coordinates
        for y in range(height):
            for x in range(width):
                pixel_value = img.getpixel((x, y))
                if pixel_value < 200:  # Check for non-white pixel
                    obstacle_coordinates.append((x, y))
        
        return obstacle_coordinates
    
    except Exception as e:
        print("An error occurred:", str(e))
        return None

# Provide the path to the input grayscale image
input_image_path = "/home/ggy/123/src/ros_foxy/maps/my_map_save.jpg"

# Get the obstacle coordinates
obstacle_coordinates = convert_to_obstacle_coordinates(input_image_path)

if obstacle_coordinates is not None:
    print("Obstacle coordinates generated successfully:")
    #for coord in obstacle_coordinates:
    #    print(coord)

    # Extract x and y coordinates for plotting
    x_coords, y_coords = zip(*obstacle_coordinates)

    # Create a scatter plot using Matplotlib
    #plt.scatter(x_coords, y_coords, s=10, c='red', marker='s')
    #plt.title('Obstacle Coordinates')
    #plt.xlabel('X')
    #plt.ylabel('Y')
    #plt.gca().invert_yaxis()  # Invert y-axis to match image coordinates
    #plt.show()

print(np.array(obstacle_coordinates).shape)

obstacle = np.array(obstacle_coordinates)


class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

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
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

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
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
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
        #print("min_x:", self.min_x)
        #print("min_y:", self.min_y)
        #print("max_x:", self.max_x)
        #print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        #self.x_width = 100
        #self.y_width = 100
        #print("x_width:", self.x_width)
        #print("y_width:", self.y_width)

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


class PathPublisherNode(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.goal_subscription = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callbacK, 1)
        #self.map_subscription = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 1)
        self.tf_subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            1)
        self.path_publisher = self.create_publisher(Path, '/robot_path', 1)
        self.p_controller = self.create_publisher(Twist, '/cmd_vel', 1)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.path = Path()

        self.sx = -2.0  # [m]
        self.sy = -0.5  # [m]
        self.gx = 0.7 # [m]
        self.gy = 0.7  # [m]
        self.grid_size = 0.5 # [m]
        self.robot_radius = 0.7 # [m]

        self.ox = ((obstacle[:,0] * 0.05)) - 9.32
        self.oy = -(((obstacle[:,1] * 0.05)) - 6.68)

        self.yaw = 0.0

    
        self.a_star = AStarPlanner(self.ox, self.oy, self.grid_size, self.robot_radius)
        self.rx, self.ry = self.a_star.planning(self.sx, self.sy, self.gx, self.gy)

        self.ready = False
        

    def odom_callback(self, msg):
        pose = msg.pose.pose
        
        self.sx = pose.position.x
        self.sy = pose.position.y

        qw = pose.orientation.w
        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z

        self.yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


    def goal_pose_callbacK(self, msg):
        self.gx = msg.pose.position.x
        self.gy = msg.pose.position.y
        self.ready = False

    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin

        self.ox = []
        self.oy = []

        #print(origin.position.x - 2.0, origin.position.y - 0.5)

        for y in range(height):
            for x in range(width):
                index = y * width + x
                cell_value = msg.data[index]

                if cell_value >= 50:
                    obstacle_x = origin.position.x + (x + 0.5) * resolution - self.map_to_odom_x
                    obstacle_y = origin.position.y + (y + 0.5) * resolution - self.map_to_odom_y
                    self.ox.append(obstacle_x)
                    self.oy.append(obstacle_y)

                

            #print(np.array(self.ox).shape)

    def tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                self.map_to_odom_x = transform.transform.translation.x
                self.map_to_odom_y = transform.transform.translation.y

    def timer_callback(self):
        

        del self.a_star
        self.a_star = AStarPlanner(self.ox, self.oy, self.grid_size, self.robot_radius)
        if (self.ready == False):
            self.rx, self.ry = self.a_star.planning(self.sx, self.sy, self.gx, self.gy)
        self.ready = True
        plt.cla()

        show_animation = 0
        if show_animation:  # pragma: no cover
            plt.plot(self.ox, self.oy, ".k")
            plt.plot(self.sx, self.sy, "og")
            plt.plot(self.gx, self.gy, "xb")
            plt.grid(True)
            plt.axis("equal")
            plt.plot(self.rx, self.ry, "-r")
            plt.pause(0.001)
            #plt.show()
            plt.draw()

        #print(np.array(self.rx).shape)

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'

        for x,y in zip(self.rx, self.ry):
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = 'odom'
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            path_msg.poses.append(pose_stamped)

        #print(self.ry)

        self.path_publisher.publish(path_msg)   
        

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
