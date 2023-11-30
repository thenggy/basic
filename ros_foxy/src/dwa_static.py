#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from rclpy.node import Node
import numpy as np
import math
from enum import Enum
import matplotlib.pyplot as plt
from tf2_msgs.msg import TFMessage

show_animation = False

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

print(obstacle.shape)


def dwa_control(x, config, goal, ob):
    """
    Dynamic Window Approach control
    """
    dw = calc_dynamic_window(x, config)

    u, trajectory = calc_control_and_trajectory(x, dw, config, goal, ob)

    return u, trajectory


class RobotType(Enum):
    circle = 0
    rectangle = 1


class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 0.26  # [m/s]
        self.min_speed = -0.1 # [m/s]
        self.max_yaw_rate = 40 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.26 # [m/ss]
        self.max_delta_yaw_rate = 40.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.02  # [m/s]   
        self.yaw_rate_resolution = 3.0 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 0.5 # [s]
        self.to_goal_cost_gain = 0.1
        self.speed_cost_gain = 0.5
        self.obstacle_cost_gain = 1.0
        self.robot_stuck_flag_cons = 0.01  # constant to prevent robot stucked
        self.robot_type = RobotType.circle

        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 0.7 # [m] for collision check

        # if robot_type == RobotType.rectangle
        self.robot_width = 1.0 # [m] for collision check
        self.robot_length = 1.2  # [m] for collision check
        # obstacles [x(m) y(m), ....]

    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value


def motion(x, u, dt):
    """
    motion model
    """

    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]

    return x


def calc_dynamic_window(x, config):
    """
    calculation dynamic window based on current state x
    """

    # Dynamic window from robot specification
    Vs = [config.min_speed, config.max_speed,
          -config.max_yaw_rate, config.max_yaw_rate]

    # Dynamic window from motion model
    Vd = [x[3] - config.max_accel * config.dt,
          x[3] + config.max_accel * config.dt,
          x[4] - config.max_delta_yaw_rate * config.dt,
          x[4] + config.max_delta_yaw_rate * config.dt]

    #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw


def predict_trajectory(x_init, v, y, config):
    """
    predict trajectory with an input
    """

    x = np.array(x_init)
    trajectory = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)
        trajectory = np.vstack((trajectory, x))
        time += config.dt

    return trajectory


def calc_control_and_trajectory(x, dw, config, goal, ob):
    """
    calculation final input with dynamic window
    """

    x_init = x[:]
    min_cost = float("inf")
    best_u = [0.0, 0.0]
    best_trajectory = np.array([x])

    # evaluate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], config.v_resolution):
        for y in np.arange(dw[2], dw[3], config.yaw_rate_resolution):

            trajectory = predict_trajectory(x_init, v, y, config)
            # calc cost
            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
            speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
            ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, ob, config)

            final_cost = to_goal_cost + speed_cost + ob_cost

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                best_u = [v, y]
                best_trajectory = trajectory
                if abs(best_u[0]) < config.robot_stuck_flag_cons \
                        and abs(x[3]) < config.robot_stuck_flag_cons:
                    # to ensure the robot do not get stuck in
                    # best v=0 m/s (in front of an obstacle) and
                    # best omega=0 rad/s (heading to the goal with
                    # angle difference of 0)
                    best_u[1] = -config.max_delta_yaw_rate
    return best_u, best_trajectory


def calc_obstacle_cost(trajectory, ob, config):
    """
    calc obstacle cost inf: collision
    """
    ox = ob[:,0] 
    oy = ob[:,1] 
    dx = trajectory[:, 0] - ox[:, None]
    dy = trajectory[:, 1] - oy[:, None]
    r = np.hypot(dx, dy)

    if config.robot_type == RobotType.rectangle:
        yaw = trajectory[:, 2]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        rot = np.transpose(rot, [2, 0, 1])
        local_ob = ob[:, None] - trajectory[:, 0:2]
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        try:
            local_ob = np.array([local_ob @ x for x in rot])
        except:
            print("No point!")
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        upper_check = local_ob[:, 0] <= config.robot_length / 2
        right_check = local_ob[:, 1] <= config.robot_width / 2
        bottom_check = local_ob[:, 0] >= -config.robot_length / 2
        left_check = local_ob[:, 1] >= -config.robot_width / 2
        if (np.logical_and(np.logical_and(upper_check, right_check),
                           np.logical_and(bottom_check, left_check))).any():
            return float("Inf")
    elif config.robot_type == RobotType.circle:
        if np.array(r <= config.robot_radius).any():
            return float("Inf")
    
    try:
        min_r = np.min(r)
        return 1.0 / min_r  # OK
    except:
        return 0.0;


def calc_to_goal_cost(trajectory, goal):
    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    error_angle = math.atan2(dy, dx)
    cost_angle = error_angle - trajectory[-1, 2]
    cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

    return cost


def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)


def plot_robot(x, y, yaw, config):  # pragma: no cover
    if config.robot_type == RobotType.rectangle:
        outline = np.array([[-config.robot_length / 2, config.robot_length / 2,
                             (config.robot_length / 2), -config.robot_length / 2,
                             -config.robot_length / 2],
                            [config.robot_width / 2, config.robot_width / 2,
                             - config.robot_width / 2, -config.robot_width / 2,
                             config.robot_width / 2]])
        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                         [-math.sin(yaw), math.cos(yaw)]])
        outline = (outline.T.dot(Rot1)).T
        outline[0, :] += x
        outline[1, :] += y
        plt.plot(np.array(outline[0, :]).flatten(),
                 np.array(outline[1, :]).flatten(), "-k")
    elif config.robot_type == RobotType.circle:
        circle = plt.Circle((x, y), config.robot_radius, color="b")
        plt.gcf().gca().add_artist(circle)
        out_x, out_y = (np.array([x, y]) +
                        np.array([np.cos(yaw), np.sin(yaw)]) * config.robot_radius)
        plt.plot([x, out_x], [y, out_y], "-k")

def distance(x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

class dwa_ros(Node):
    def __init__(self):
        super().__init__('Odometry_Node')
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        #self.map_subscription = self.create_subscription(
        #    OccupancyGrid,
        #   '/map',
        #    self.map_callback,
        #    1)
            
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            1)
        
        self.tf_subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            1)
        
        self.point_goal_publisher = self.create_publisher(PointStamped, '/current_goal', 1)
        
        self.path_subscription = self.create_subscription(
            Path,
            '/robot_path',
            self.path_callback,
            1)
        
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        #x,y,yaw(rad),V, Wz
        self.x = np.array([-2.0, -0.5, 0.0, 0.0, 0.0])
        self.goal = np.array([0.7, 0.7])
        
        self.config = Config()
        self.config.robot_type = RobotType.rectangle
        self.trajectory = np.array(self.x)

        self.x_pos = 0.0
        self.y_pos = 0.0
        self.yaw = 0.0

        self.map_to_odom_x = 0.0
        self.map_to_odom_y = 0.0

        self.robot_path = Path()

        self.rx = [0.0]
        self.ry = [0.0]

        self.ox = ((obstacle[:,0] * 0.05)) - 9.32
        self.oy = -(((obstacle[:,1] * 0.05)) - 6.68)

        OBSTACLE = np.column_stack((self.ox, self.oy))

        print(OBSTACLE.shape)

        self.config.ob = OBSTACLE # Convert to list
        self.ob = self.config.ob

    def timer_callback(self):

        # Iterate through the path points in reverse order
        if len(self.rx) > 2:
            for i in range(len(self.rx)):
                distance_to_trajectory = distance(self.x_pos, self.y_pos, self.rx[i], self.ry[i])
                print(i)
                
                if distance_to_trajectory < 2.5:
                    self.goal[0] = self.rx[i]
                    self.goal[1] = self.ry[i]
                    print(self.x_pos, self.y_pos, self.rx[i], self.ry[i])
                    break  # Exit the loop after finding the next goal
        else:
            self.goal[0] = self.rx[0]
            self.goal[1] = self.ry[0]
        
        self.u, predicted_trajectory = dwa_control(self.x, self.config, self.goal, self.config.ob)
        self.x = motion(self.x, self.u, self.config.dt)
        self.trajectory = np.vstack((self.trajectory, self.x))

        show_animation = 0

        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
            plt.xlim(-2, 4)  # Set fixed x-axis limits
            plt.ylim(-2, 4)  # Set fixed y-axis limits
            plt.plot(self.x[0], self.x[1], "xr")
            plt.plot(self.goal[0], self.goal[1], "xb")
            plt.plot(self.ob[:, 0], self.ob[:, 1], "ok")
            plot_robot(self.x[0], self.x[1], self.x[2], self.config)
            plot_arrow(self.x[0], self.x[1], self.x[2])

            
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)
        
        cmd_msg = Twist()
        cmd_msg.linear.x = self.u[0]
        cmd_msg.angular.z = self.u[1]
        
        # check reaching goal
        dist_to_goal = math.hypot(self.x[0] - self.goal[0], self.x[1] - self.goal[1])
        if dist_to_goal <= self.config.robot_radius:
            print("Goal!!")
            cmd_msg = Twist()

        self.publisher.publish(cmd_msg)

        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = 'odom'

        point_msg.point.x = self.goal[0]
        point_msg.point.y = self.goal[1] 
        point_msg.point.z = 0.0
        self.point_goal_publisher.publish(point_msg)

        #print(self.goal[0], self.goal[1])
    
    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin

        obstacle = []
        for y in range(height):
            for x in range(width):
                index = y * width + x
                cell_value = msg.data[index]

                if cell_value >= 50:
                    obstacle_x = origin.position.x + (x + 0.5) * resolution - self.map_to_odom_x
                    obstacle_y = origin.position.y + (y + 0.5) * resolution - self.map_to_odom_y

                    if distance(obstacle_x, obstacle_y, self.x_pos, self.y_pos) < 10.0:
                        obstacle.append([obstacle_x, obstacle_y])

        # Remove points that are 0.1 meters close to each other
        i = 0

        """
        while i < len(obstacle):
            j = i + 1
            while j < len(obstacle):
                if distance(obstacle[i][0], obstacle[i][1], obstacle[j][0], obstacle[j][1]) < 0.1:
                    del obstacle[j]
                else:
                    j += 1
            i += 1
        """

        self.config.ob = np.array(obstacle) # Convert to list
        self.ob = self.config.ob


    def tf_callback(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                self.map_to_odom_x = transform.transform.translation.x
                self.map_to_odom_y = transform.transform.translation.y

    def odom_callback(self, msg):
        pose = msg.pose.pose
        twist = msg.twist.twist

        self.x_pos = pose.position.x
        self.y_pos = pose.position.y

        qx = pose.orientation.x
        qy = pose.orientation.y
        qz = pose.orientation.z
        qw = pose.orientation.w
        
        Vx = twist.linear.x
        Vy = twist.linear.y

        # Convert quaternion to yaw using atan2 and trigonometric functions
        self.yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        self.x[0] = self.x_pos
        self.x[1] = self.y_pos
        self.x[2] = self.yaw

    def path_callback(self,msg):
        self.robot_path = msg
        self.rx = [msg.poses[i].pose.position.x for i in range(len(msg.poses))]
        self.ry = [msg.poses[i].pose.position.y for i in range(len(msg.poses))]


def main(args=None):
    rclpy.init(args=args)
    dwa_ros_node = dwa_ros()
    rclpy.spin(dwa_ros_node)
    dwa_ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    	
    	
    	
        
