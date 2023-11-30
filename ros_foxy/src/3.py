import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs

class TFListenerNode(Node):

    def __init__(self):
        super().__init__('tf_listener_node')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def transform_robot_pose_to_map_frame(self):
        robot_pose_in_base_frame = PoseStamped()
        robot_pose_in_base_frame.header.frame_id = "base_link"  # Replace with your robot's base frame
        robot_pose_in_base_frame.pose.position.x = 0.0  # Replace with your robot's actual pose
        robot_pose_in_base_frame.pose.position.y = 0.0
        robot_pose_in_base_frame.pose.orientation.w = 1.0

        try:
            # Wait for the transformation to become available (blocking)
            self.tf_buffer.can_transform("map", "base_link", rclpy.time.Time(), rclpy.duration.Duration(1.0))

            # Transform the robot's pose to the map frame
            robot_pose_in_map_frame = self.tf_buffer.transform(robot_pose_in_base_frame, "map")

            # Now, robot_pose_in_map_frame contains the robot's pose in the map frame
            self.get_logger().info(f"Robot's pose in map frame: {robot_pose_in_map_frame}")
        except Exception as e:
            self.get_logger().error(f"Failed to transform robot pose: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    tf_listener_node = TFListenerNode()
    tf_listener_node.transform_robot_pose_to_map_frame()
    rclpy.spin(tf_listener_node)
    tf_listener_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
