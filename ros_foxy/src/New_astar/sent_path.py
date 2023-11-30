#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


# from mte544_action_interfaces.action import Move2Goal

class AStarClient(Node):

    def __init__(self):
        super().__init__('a_star_client')
        # self._action_client = ActionClient(self, Move2Goal, 'mte_544_a_star')
        # create the subscriber object to RViz `2D Goal Pose`
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_pose_rviz_callback, 1)
        
        # self.p_controller = self.create_publisher(Twist, '/cmd_vel', 1)
        self.Pose_Estimate_publish = self.create_publisher(PoseWithCovarianceStamped, '/abc',1)
        # Used for finding TF between base_link frame and map (i.e. robot position)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.initial_pose = None
        self.goal_pose = None

        # ROS2 params for setting goal through node launch
        self.declare_parameter('predefined_goal', False)
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        
      
    def get_current_pose(self):
        """Get current pose of Turtlebot"""
        # Find the transform between base_link and map
        from_frame_rel = 'base_link'
        to_frame_rel = 'map'
        for i in range(500):
            # rclpy.spin_once(self)
            print(i)
            try:
                t = self.tf_buffer.lookup_transform(
                    to_frame_rel,
                    from_frame_rel,
                    rclpy.time.Time())
                curr_pose_x = t.transform.translation.x
                curr_pose_y = t.transform.translation.y
                base_map_rot = t.transform.rotation
                quaternion = (
                    base_map_rot.x,
                    base_map_rot.y,
                    base_map_rot.z,
                    base_map_rot.w
                )

                initial_pose = PoseWithCovarianceStamped()
                initial_pose.pose.pose.position.x = curr_pose_x
                initial_pose.pose.pose.position.y = curr_pose_y
                initial_pose.pose.pose.orientation.x = quaternion[0]
                initial_pose.pose.pose.orientation.y = quaternion[1]
                initial_pose.pose.pose.orientation.z = quaternion[2]
                initial_pose.pose.pose.orientation.w = quaternion[3]
                return initial_pose

            except TransformException as ex:
                print(f"TransformException: {ex}")
                pass

    def goal_pose_rviz_callback(self, msg):
        """Store `2D Goal Pose` from RViz"""
        # if self.goal_pose is None :
        self.goal_pose = msg
        self.get_logger().info(f"Goal position: {self.goal_pose.pose.position.x, self.goal_pose.pose.position.y}")
        self.send_goal()


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
        
        
 
    
def main(args=None):
    rclpy.init(args=args)

    action_client = AStarClient()
    
   

    # if action_client.get_parameter('predefined_goal').value:
    #     # Receiving a goal pose through node launch
    #     # Therefore, get initial pose of robot ourselves
    #     initial_pose = action_client.get_current_pose()
    
    # if action_client.get_parameter('predefined_goal').value and initial_pose is not None:
    #     # Ensure we were able to get initial pose of robot ourselves        
        
    #     # print("hi")
    #     goal_x = action_client.get_parameter('goal_x').value
    #     goal_y = action_client.get_parameter('goal_y').value
    #     initial_x = initial_pose.pose.pose.position.x
    #     initial_y = initial_pose.pose.pose.position.y
    #     action_client.send_goal(initial_x, initial_y, goal_x, goal_y)
    # else:
    #     # Goal will be received through RViz
    #     # print("hello")
    #     action_client.send_goal()
    # action_client.send_goal()
    try:
        rclpy.spin(action_client) 
    except KeyboardInterrupt:
        pass
    
    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
