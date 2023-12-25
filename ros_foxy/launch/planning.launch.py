#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix
from launch.substitutions import LaunchConfiguration


# ROBOT= os.environ['ROBOT']

def generate_launch_description():

    lifecycle_nodes=['map_server'
                     ]
    initial_pose_x = 0.0 #2
    initial_pose_y = 0.0 #0.5
    initial_yaw = 0
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_box_bot_gazebo = get_package_share_directory('ros_foxy')
    launch_file_dir = os.path.join(get_package_share_directory('ros_foxy'), 'launch')
    # world_file_name= ROBOT +'.world'
    # We get the whole install dir
    # We do this to avoid having to copy or softlink manually the packages so that gazebo can find them
    description_package_name = "ros_foxy"
    install_dir = get_package_prefix(description_package_name)
    
    params_file = os.path.join(get_package_share_directory("ros_foxy"), 'config', 'burger.yaml')

    # Set the path to the WORLD model files. Is to find the models inside the models folder in my_box_bot_gazebo package
    gazebo_models_path = os.path.join(pkg_box_bot_gazebo, 'models')
    # os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    map_dir = LaunchConfiguration(
        'maps',
        default=os.path.join(
            get_package_share_directory('ros_foxy'),
            'maps',
            'my_map_save.yaml')) # 1111_map.yaml  , x_map002.yaml , map_vision_ai.yaml

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )    

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'maps',
            default_value=map_dir,
            description='Full path to map file to load'),
        # DeclareLaunchArgument(
        #   'world',
        #   default_value=[os.path.join(pkg_box_bot_gazebo, 'world', 'world12.sdf')],
        #   description='SDF world file'),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([launch_file_dir, '/rviz.launch.py']),
        #     launch_arguments={'use_sim_time': use_sim_time}.items(),
        # ),
        
        # Node(
        #     package='ros_foxy',
        #     executable='astar_single.py',
        #     output='screen',
        #     name='path_publisher'
        # ),
        # Node(
        #     package='ros_foxy',
        #     executable='dwa_static.py',
        #     output='screen',
        #     name='Odometry_Node'
        # ),
        # Node(
        #         package='nav2_map_server',
        #         executable='map_server',
        #         name='map_server',
        #         output='screen',
        #         parameters=[{'yaml_filename': map_dir},{'use_sim_time': True}]
        #         ),
        
        launch.actions.TimerAction(
			period=1.5,
			actions=[
				launch_ros.actions.Node(
					package='nav2_map_server',
					executable='map_server',
					name='map_server',
					output='screen',
					parameters=[{'yaml_filename': map_dir}]
				),
			],
		),
        Node(
                package = "tf2_ros", 
                executable = "static_transform_publisher",
                arguments = [str(initial_pose_x), str(initial_pose_y), "0", str(initial_yaw), "0", "0", "map", "odom"]
                ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes},
                        # {'params_file':params_file},
                        ]
            ),
        # launch_ros.actions.Node(
        #     package='nav2_planner',
        #     executable='planner_server',
        #     name='planner_server',
        #     output='screen',
        #     parameters=[{params_file}]
		# ),
                            
                # gazebo
    ])
