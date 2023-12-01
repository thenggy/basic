import launch
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

	# Default initial robot positions for gazebo simulation.
	# Should be modified to match robot's actual position as needed.
	initial_pose_x = 0.0 #2
	initial_pose_y = 0.0 #0.5
	initial_yaw = 0

	lifecycle_nodes = [
		'map_server',
		'planner_server',
		'amcl',
	]
	launch_file_dir = os.path.join(get_package_share_directory('ros_foxy'), 'launch')

	map_file = os.path.join(get_package_share_directory("ros_foxy"), 'maps', 'map_vision_ai.yaml')
	params_file = os.path.join(get_package_share_directory("ros_foxy"), 'config', 'localization.yaml')
	# nav2_yaml = os.path.join(get_package_share_directory('ros_foxy'),'param','localization.yaml')
	
	return launch.LaunchDescription([
		
		# Start map_server and lifecycle node after 1.5 seconds 
		launch.actions.TimerAction(
			period=1.5,
			actions=[
				launch_ros.actions.Node(
					package='nav2_map_server',
					executable='map_server',
					name='map_server',
					output='screen',
					parameters=[{'yaml_filename': map_file}]
				),
			],
		),

        launch_ros.actions.Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[{params_file}]
		),
		
		launch_ros.actions.Node(
			package = "tf2_ros", 
			executable = "static_transform_publisher",
			arguments = [str(initial_pose_x), str(initial_pose_y), "0", str(initial_yaw), "0", "0", "map", "odom"]
		),
  		# launch_ros.actions.Node(
		# 	package = "tf2_ros", 
		# 	executable = "static_transform_publisher",
		# 	arguments = ['0', '0', "0", '0.0', "0.0", "0.0", "odom", "base_link"]
		# ),

        launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_planning',
            output='screen',
            parameters=[
				{'use_sim_time':False},
				{'autostart': True},
				{'node_names': lifecycle_nodes},
                {'params_file':params_file},
			]),
		# launch_ros.actions.Node(
		# 	package='rviz2',
		# 	executable='rviz2',
		# 	arguments=['-d', [os.path.join(get_package_share_directory("mte544_a_star"), 'launch')]]
		# ),
  
		        launch_ros.actions.Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file]
        ),

		# launch_ros.actions.Node(
		# 	package='mte544_a_star',
		# 	executable='mte544_navigation_server.py',
		# ),
		# IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([launch_file_dir, '/gazebo.launch.py']),
        #     # launch_arguments={'use_sim_time': use_sim_time}.items(),
        # ),
    ])
