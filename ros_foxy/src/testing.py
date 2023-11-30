import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

import math

import matplotlib.pyplot as plt

show_animation = True

class OccupancyGridSubscriber(Node):

    def __init__(self):
        super().__init__('occupancy_grid_subscriber')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',  # Replace with the actual topic name
            self.occupancy_grid_callback,
            10  # Adjust the queue size as needed
        )
        
        self.subscription  # prevent unused variable warning

    def occupancy_grid_callback(self, msg):
    # Access the occupancy grid data as a list
        occupancy_grid_data = msg.data

        # Access the width and height of the occupancy grid
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin

        # Process the data, width, and height as needed
        self.get_logger().info(f'Received occupancy grid data with width: {width}, height: {height}')

        # For example, you can iterate through the data:
        for y in range(height):
            for x in range(width):
                index = x + y * width
                cell_value = occupancy_grid_data[index]
                
                # Calculate real-world coordinates (x, y) from the grid cell indices
                map_x = origin.position.x + (x + 0.5) * resolution
                map_y = origin.position.y + (y + 0.5) * resolution

                # Process the cell value (occupancy probability) here
                # For instance, you can check if it's occupied (cell_value >= 40),
                # free (cell_value == 0), or unknown (other values in between).
                if cell_value >= 0.65:  # Cell is occupied
                    print(f'Obstacle at (x={map_x}, y={map_y})')
                elif cell_value <= 0.25:  # Cell is free
                    print(f'Free space at (x={map_x}, y={map_y})')
                else:  # Cell is unknown
                    print(f'Unknown at (x={map_x}, y={map_y})')

def main(args=None):
    rclpy.init(args=args)
    subscriber_node = OccupancyGridSubscriber()
    rclpy.spin(subscriber_node)
    subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
