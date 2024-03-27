#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2022, ETFSA
# All rights reserved.
#
# Author: Dinko Osmankovic <dinko.osmankovic@etf.unsa.ba>
# Modified: Nermin Covic <nermin.covic@etf.unsa.ba>

# ros2 launch realsense2_camera rs_multi_camera_launch.py camera_name1:=D400 device_type2:=l5. device_type1:=d4..

import copy
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
    

def generate_launch_description():
    camera_left_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
        launch_arguments={
            #'initial_reset': 'true',
            'camera_name': 'camera_left',
            'serial_no': "_036222070643",
            'pointcloud.enable': 'true',
            'spatial_filter.enable': 'true',
            'temporal_filter.enable': 'true',
        }.items(),
    )

    camera_right_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])),
        launch_arguments={
            #'initial_reset': 'true',
            'camera_name': 'camera_right',
            'serial_no': "_036522072967",
            'pointcloud.enable': 'true',
            'spatial_filter.enable': 'true',
            'temporal_filter.enable': 'true',
        }.items(),
    )

    ########################################################################
    # # Nermin (calibrated by hand):
    tf_node_world_link_base = Node(package = "tf2_ros", 
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0", "0", "0", "0", "world", "link_base"]
    )

    tf_node_link_base_aruco_marker = Node(package = "tf2_ros", 
            executable = "static_transform_publisher",
            arguments = ["0.43", "0", "0", "1.57079", "0", "0", "link_base", "aruco_marker"]
    )

    tf_node_aruco_marker_camera_left_link = Node(package = "tf2_ros", 
            name="left_transform",
            executable = "static_transform_publisher",
            arguments = ["-1.14", "0", "0.92", "0.37", "0.67", "0.06", \
                        "aruco_marker", "camera_left_link"]     # (x,y,z, yaw(z), pich(y), roll(x))
	)

    tf_node_aruco_marker_from_right_camera_right_link = Node(package = "tf2_ros", 
            name="right_transform",
            executable = "static_transform_publisher",
            arguments = ["0.85", "0", "1.19", "0.2", "2.2", "0.2", \
                        "aruco_marker_from_right", "camera_right_link"]    # (x,y,z, yaw(z), pich(y), roll(x))
	)

    tf_node_aruco_marker_aruco_marker_from_right = Node(package = "tf2_ros",
            executable = "static_transform_publisher",
            arguments = ["0", "-0.06", "0", "3.14159", "-1.57079", "0", \
                        "aruco_marker", "aruco_marker_from_right"]
	)
    ########################################################################

    # Nermin (calibrated using aruco_calibration package):
    # tf_node_world_link_base = Node(package = "tf2_ros", 
    #         executable = "static_transform_publisher",
    #         arguments = ["0", "0", "0", "0", "0", "0", "world", "link_base"]
    # )

    # # Needs to measure the distance on the table
    # tf_node_link_base_aruco_marker = Node(package = "tf2_ros", 
    #         executable = "static_transform_publisher",
    #         arguments = ["0.5", "0", "0", "1.57079", "0", "0", "link_base", "aruco_marker"]
    # )

    # # Readings from aruco calibration package
    # tf_node_aruco_marker_camera_left = Node(package = "tf2_ros", 
    #         name="left_transform",
    #         executable = "static_transform_publisher",
    #         arguments = ["0.480", "0.054", "1.363", "-1.059", "-0.832", "2.681", \
    #                     "aruco_marker", "camera_left"]     # (x,y,z, yaw(z), pich(y), roll(x))
	# )

    # # Needs to correct the transformation in order to align with camera_left_color_optical_frame
    # tf_node_camera_left_camera_left_link = Node(package = "tf2_ros",
    #         executable = "static_transform_publisher",
    #         arguments = ["0", "0", "0", "-1.57079", "0", "-1.57079", \
    #                     "camera_left", "camera_left_link"]     # (x,y,z, yaw(z), pich(y), roll(x))
	# )
    
    # rviz_pkg = get_package_share_directory('sim_bringup')
    # default_rviz_config_path = os.path.join(rviz_pkg, 'rviz/etflab.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        # arguments=['-d', default_rviz_config_path],
        # parameters=[{'use_sim_time': use_sim_time}]
    )

    pointcloud_combiner_node = Node(
        package='perception_etflab',
        executable='pointcloud_combiner',
        name='pointcloud_combiner',
        output='screen',
        parameters=[
            # {"point_cloud_topics": ["/camera_left/depth/color/points", "/camera_right/depth/color/points"]},
            {"point_cloud_topics": ["/camera_left/depth/color/points"]},
            # {"point_cloud_topics": ["/camera_right/depth/color/points"]},
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
        camera_left_node,
        camera_right_node,
        tf_node_world_link_base,
        tf_node_link_base_aruco_marker,
        tf_node_aruco_marker_camera_left_link,
        # tf_node_aruco_marker_aruco_marker_from_right,
        # tf_node_aruco_marker_from_right_camera_right_link,
        
        # tf_node_aruco_marker_camera_left,
        # tf_node_camera_left_camera_left_link,
        rviz_node,
        TimerAction(
            period=1.0,
            actions=[pointcloud_combiner_node]
        ),
        TimerAction(
            period=2.0,
            actions=[object_segmentation_node]
        ),
        # TimerAction(
        #     period=3.0,
        #     actions=[octomap_server_node]
        # ),
    ])
