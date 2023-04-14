# Authors: Abdulkadir Ture
# Github : abdulkadrtr

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_folder = 'turtlebot3_burger'
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    robot_desc_path = os.path.join(get_package_share_directory("turtlebot3_gazebo"), "urdf", "turtlebot3_burger.urdf")
    world = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'worlds','turtlebot3_house.world')
    urdf_path1 = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'models',model_folder+'_0','model.sdf')
    urdf_path2 = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'models',model_folder+'_1','model.sdf')
    with open(robot_desc_path, 'r') as infp:
        robot_desc = infp.read()
    name1 = "tb3_0"
    name2 = "tb3_1"
    # 1. ROBOT ICIN KODLAR
    spawn_robot1 = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py', 
        arguments=[
            '-entity', name1, 
            '-file', urdf_path1, 
            '-x', '5.0', 
            '-y', '2.5', 
            '-z', '0.01',
            '-robot_namespace', name1,
        ],
        output='screen'
    )
    robot_state_publisher1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=name1,
        output='screen',
        parameters=[{'frame_prefix': name1 + '/',
                    'use_sim_time': True,
                    'robot_description': robot_desc}]
    )
    async_slam_toolbox1 = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='async_slam_toolbox_node',
        namespace=name1,
        parameters=[{
            'use_sim_time': True,
            'odom_frame': name1 + '/odom',
            'base_frame': name1 + '/base_footprint',
            'scan_topic': 'scan',
            'map_frame': name1 + '/map',
            'minimum_travel_distance': 0.3,
            'minimum_travel_heading': 0.3,
            'resolution': 0.05,
        }],
        remappings=[
            ("/map", "map"),
            ("/map_metadata", "map_metadata"),
            ("/slam_toolbox/scan_visualization", "slam_toolbox/scan_visualization"),
            ("/slam_toolbox/graph_visualization", "slam_toolbox/graph_visualization"),
        ],
        output='screen',
    )  
    rviz1 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'config',name1+'.rviz')]
    )
    # 2. ROBOT ICIN KODLAR

    spawn_robot2 = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py', 
        arguments=[
            '-entity', name2, 
            '-file', urdf_path2, 
            '-x', '-4.6', 
            '-y', '3.0', 
            '-z', '0.01',
            '-robot_namespace', name2,
        ],
        output='screen'
    )
    robot_state_publisher2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=name2,
        output='screen',
        parameters=[{'frame_prefix': name2 + '/',
                    'use_sim_time': True,
                    'robot_description': robot_desc}]
    )
    async_slam_toolbox2 = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='async_slam_toolbox_node',
        namespace=name2,
        parameters=[{
            'use_sim_time': True,
            'odom_frame': name2 + '/odom',
            'base_frame': name2 + '/base_footprint',
            'scan_topic': 'scan',
            'map_frame': name2 + '/map',
            'minimum_travel_distance': 0.3,
            'minimum_travel_heading': 0.3,
            'resolution': 0.05,
        }],
        remappings=[
            ("/map", "map"),
            ("/map_metadata", "map_metadata"),
            ("/slam_toolbox/scan_visualization", "slam_toolbox/scan_visualization"),
            ("/slam_toolbox/graph_visualization", "slam_toolbox/graph_visualization"),
        ],
        output='screen',
    )  
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'config', name2+'.rviz')]
    )
    # GAZEBO ICIN KODLAR
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world,'verbose':"true",'extra_gazebo_args': 'verbose'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        launch_arguments={'verbose':"true"}.items()
    )    
    pathFollow1 = Node(
        package='path_follow',
        executable='path_follow',
        name='pathFollow',
        output='screen',
        namespace=name1,
        parameters=[{
            'use_sim_time': True,
        }],
        remappings=[
            ("/path", "/"+name1+"/path"),
            ("/cmd_vel", "/"+name1+"/cmd_vel"),
            ("/odom", "/"+name1+"/odom"),
            ("/visual_path", "/"+name2+"/visual_path"),
        ]
    )
    pathFollow2 = Node(
        package='path_follow',
        executable='path_follow',
        name='pathFollow',
        output='screen',
        namespace=name2,
        parameters=[{
            'use_sim_time': True,
        }],
        remappings=[
            ("/path", "/"+name2+"/path"),
            ("/cmd_vel", "/"+name2+"/cmd_vel"),
            ("/odom", "/"+name2+"/odom"),
            ("/visual_path", "/"+name1+"/visual_path"),
        ]
    )
    sjtu_drone_bringup_path = get_package_share_directory('sjtu_drone_bringup')
    drone = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sjtu_drone_bringup_path, 'launch', 'sjtu_drone_gazebo.launch.py')
        ),
    )
    drone_control = Node(
        package='drone_control',
        executable='drone_control',
        name='drone_control',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
    )
    ld = LaunchDescription()
    ld.add_action(drone_control)
    ld.add_action(drone)
    ld.add_action(pathFollow1)
    ld.add_action(pathFollow2)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_robot1)
    ld.add_action(robot_state_publisher1)
    ld.add_action(async_slam_toolbox1)
    ld.add_action(rviz1)
    ld.add_action(spawn_robot2)
    ld.add_action(robot_state_publisher2)
    ld.add_action(async_slam_toolbox2)
    ld.add_action(rviz2)
    return ld
