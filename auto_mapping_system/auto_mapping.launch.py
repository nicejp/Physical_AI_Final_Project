#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 1. Gazebo with TurtleBot3 House environment
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_house.launch.py'
            ])
        ])
    )
    
    # 2. Cartographer SLAM
    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_cartographer'),
                'launch',
                'cartographer.launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # 3. RViz for real-time visualization
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('turtlebot3_cartographer'),
        'rviz',
        'tb3_cartographer.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # 4. Auto exploration node (delayed start to ensure other nodes are ready)
    auto_explorer_node = TimerAction(
        period=5.0,  # Wait 5 seconds before starting exploration
        actions=[
            Node(
                package='auto_mapping_system',
                executable='auto_explorer',
                name='auto_mapping_explorer',
                parameters=[{'use_sim_time': use_sim_time}],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time for all nodes'
        ),
        gazebo_launch,
        cartographer_launch,
        rviz_node,
        auto_explorer_node
    ])
