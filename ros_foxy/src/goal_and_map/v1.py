import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

class GoalPoseSubscriber(Node):

    def __init__(self):
        super().__init__('goal_pose_subscriber')
        self.subscription_goal = self.create_subscription(
            PoseStamped,
            'goal_pose',  # Replace with the actual topic name
            self.goal_pose_callback,
            10  # Adjust the queue size as needed
        )

        self.subscription_grid = self.create_subscription(
            OccupancyGrid,
            'map',  # Replace with the actual topic name
            self.occupancy_grid_callback,
            10  # Adjust the queue size as needed
        )

        self.occupancy_grid_data = None  # Store the occupancy grid data here
        self.width = 0
        self.height = 0
        self.resolution = 0.0
        self.origin = None

    def goal_pose_callback(self, msg):
        # Process the received 2D goal pose message here
        x = msg.pose.position.x
        y = msg.pose.position.y

        # Check if the occupancy grid data is available
        if self.occupancy_grid_data is not None:
            x_index = int((x - self.origin.position.x) / self.resolution)
            y_index = int((y - self.origin.position.y) / self.resolution)
            
            if x_index >= 0 and x_index < self.width and y_index >= 0 and y_index < self.height:
                index = x_index + y_index * self.width
                cell_value = self.occupancy_grid_data[index]
                
                if cell_value >= 0.65:  # Cell is occupied
                    self.get_logger().error(f'Goal pose points to an obstacle at (x={x}, y={y})')
                else:
                    self.get_logger().info(f'Goal pose is in free space at (x={x}, y={y})')
            else:
                self.get_logger().warning(f'Goal pose is outside the occupancy grid')
        else:
            self.get_logger().warning('No occupancy grid data received yet.')

    def occupancy_grid_callback(self, msg):
        # Store the occupancy grid data and related information
        self.occupancy_grid_data = msg.data
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin

def main(args=None):
    rclpy.init(args=args)
    goal_pose_subscriber_node = GoalPoseSubscriber()
    rclpy.spin(goal_pose_subscriber_node)
    goal_pose_subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
