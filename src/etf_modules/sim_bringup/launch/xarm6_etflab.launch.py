#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2022, ETFSA
# All rights reserved.
#
# Author: Dinko Osmankovic <dinko.osmankovic@etf.unsa.ba>

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable, RegisterEventHandler, LogInfo, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, Command, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit, OnProcessStart
from ament_index_python import get_package_share_directory

def generate_launch_description():
    report_type = LaunchConfiguration('report_type', default='dev')     # normal, rich, dev (see: https://github.com/xArm-Developer/xarm_ros#report_type-argument)
    dof = LaunchConfiguration('dof', default='6')
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=False)
    effort_control = LaunchConfiguration('effort_control', default=False)
    velocity_control = LaunchConfiguration('velocity_control', default=False)
    add_gripper = LaunchConfiguration('add_gripper', default=True)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)

    add_other_geometry = LaunchConfiguration('add_other_geometry', default=False)
    geometry_type = LaunchConfiguration('geometry_type', default='box')
    geometry_mass = LaunchConfiguration('geometry_mass', default=0.1)
    geometry_height = LaunchConfiguration('geometry_height', default=0.1)
    geometry_radius = LaunchConfiguration('geometry_radius', default=0.1)
    geometry_length = LaunchConfiguration('geometry_length', default=0.1)
    geometry_width = LaunchConfiguration('geometry_width', default=0.1)
    geometry_mesh_filename = LaunchConfiguration('geometry_mesh_filename', default='')
    geometry_mesh_origin_xyz = LaunchConfiguration('geometry_mesh_origin_xyz', default='"0 0 0"')
    geometry_mesh_origin_rpy = LaunchConfiguration('geometry_mesh_origin_rpy', default='"0 0 0"')
    geometry_mesh_tcp_xyz = LaunchConfiguration('geometry_mesh_tcp_xyz', default='"0 0 0"')
    geometry_mesh_tcp_rpy = LaunchConfiguration('geometry_mesh_tcp_rpy', default='"0 0 0"')
    
    camera_left_x = LaunchConfiguration('camera_left_x', default=1.5)
    camera_left_y = LaunchConfiguration('camera_left_y', default=-0.8)
    camera_left_z = LaunchConfiguration('camera_left_z', default=0.6)
    camera_left_R = LaunchConfiguration('camera_left_R', default=0)
    camera_left_P = LaunchConfiguration('camera_left_P', default=0.30)
    camera_left_Y = LaunchConfiguration('camera_left_Y', default=2.8)
    
    camera_right_x = LaunchConfiguration('camera_right_x', default=1.5)
    camera_right_y = LaunchConfiguration('camera_right_y', default=0.8)
    camera_right_z = LaunchConfiguration('camera_right_z', default=0.6)
    camera_right_R = LaunchConfiguration('camera_right_R', default=0)
    camera_right_P = LaunchConfiguration('camera_right_P', default=0.30)
    camera_right_Y = LaunchConfiguration('camera_right_Y', default=3.5)
    
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    
    # robot gazebo launch
    robot_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/_robot_spawn.launch.py']),
        launch_arguments={
            'report_type': report_type,
            'dof': dof,
            'prefix': prefix,
            'hw_ns': hw_ns,
            'limited': limited,
            'effort_control': effort_control,
            'velocity_control': velocity_control,
            'add_gripper': add_gripper,
            'add_vacuum_gripper': add_vacuum_gripper,
            'robot_type': 'xarm',
            'add_other_geometry': add_other_geometry,
            'geometry_type': geometry_type,
            'geometry_mass': geometry_mass,
            'geometry_height': geometry_height,
            'geometry_radius': geometry_radius,
            'geometry_length': geometry_length,
            'geometry_width': geometry_width,
            'geometry_mesh_filename': geometry_mesh_filename,
            'geometry_mesh_origin_xyz': geometry_mesh_origin_xyz,
            'geometry_mesh_origin_rpy': geometry_mesh_origin_rpy,
            'geometry_mesh_tcp_xyz': geometry_mesh_tcp_xyz,
            'geometry_mesh_tcp_rpy': geometry_mesh_tcp_rpy,
            'camera_left_x': camera_left_x,
            'camera_left_y': camera_left_y,
            'camera_left_z': camera_left_z,
            'camera_left_R': camera_left_R,
            'camera_left_P': camera_left_P,
            'camera_left_Y': camera_left_Y,
            'camera_right_x': camera_right_x,
            'camera_right_y': camera_right_y,
            'camera_right_z': camera_right_z,
            'camera_right_R': camera_right_R,
            'camera_right_P': camera_right_P,
            'camera_right_Y': camera_right_Y,
        }.items(),
    )
        
    rviz_pkg = get_package_share_directory('sim_bringup')
    default_rviz_config_path = os.path.join(rviz_pkg, 'rviz/etflab.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', default_rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    pointcloud_combiner_node = Node(
            package='perception_etflab',
            executable='pointcloud_combiner',
            name='pointcloud_combiner',
            output='screen',
            parameters=[
            	{"point_cloud_topics": ["/camera_left/points", "/camera_right/points"]},
            	{"output_topic": "pointcloud_combined"}
            ]
    )
    
    object_segmentation_node = Node(
        package='perception_etflab',
        executable='object_segmentation',
        name='object_segmentation',
        parameters=[
        	{'input_cloud': 'pointcloud_combined'},
        	{'objects_cloud': 'objects_cloud'}
        ]
    )
    
    octomap_server_node = Node(
        package='octomap_server',
        executable='tracking_octomap_server_node',
        name='tracking_octomap_server_node',
        output='screen',
        parameters=[
        	{'resolution': 0.05},
        	{'frame_id': 'world'},
        	{'save_directory': '$(env OCTOMAP_SAVE_DIR ./)'},
        ],
        remappings=[
            ("cloud_in", "objects_cloud")
        ]
    )
        
    return LaunchDescription([    
    	TimerAction(
            period=2.0,
            actions=[rviz_node, pointcloud_combiner_node, object_segmentation_node]
        ),
		robot_gazebo_launch,
        # TimerAction(
        #     period=4.0,
        #     actions=[octomap_server_node]
        # ),
    ])
