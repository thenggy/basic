import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalPoseSubscriber(Node):

    def __init__(self):
        super().__init__('goal_pose_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped,
            'goal_pose',  # Replace with the actual topic name
            self.goal_pose_callback,
            10  # Adjust the queue size as needed
        )
        self.subscription  # prevent unused variable warning

    def goal_pose_callback(self, msg):
        # Process the received 2D goal pose message here
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.get_logger().info(f'Received 2D Goal Pose (x, y): ({x}, {y})')

def main(args=None):
    rclpy.init(args=args)
    goal_pose_subscriber_node = GoalPoseSubscriber()
    rclpy.spin(goal_pose_subscriber_node)
    goal_pose_subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
