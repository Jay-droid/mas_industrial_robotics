"""
Copyright 2022 Bonn-Rhein-Sieg University

Author: Vamsi Kalagaturu, Vivek Mannava

"""

from launch import LaunchDescription
from launch.actions import (
    RegisterEventHandler,
    EmitEvent,
    LogInfo,
    DeclareLaunchArgument,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers.on_shutdown import OnShutdown

import lifecycle_msgs.msg
import os


def generate_launch_description():
    ld = LaunchDescription()
    node_name = "mmor"

    # 1. Define fallbacks / defaults using standard directory patterns
    default_scene_seg_constraints = os.path.join(
        get_package_share_directory("mir_object_recognition"),
        "ros",
        "config",
        "scene_segmentation_constraints.yaml",
    )

    default_objects_info = os.path.join(
        get_package_share_directory("mir_object_recognition"),
        "ros",
        "config",
        "objects.yaml",
    )

    default_yolo_classes_info = os.path.join(
        get_package_share_directory("mir_object_recognition"),
        "ros",
        "config",
        "yolo_inference",
        "ss22_competition.yaml",
    )

    default_yolo_weights = "/home/batsy/Downloads/ss22_competition_best.onnx"

    # 2. Declare the input Launch Arguments
    objects_info_arg = DeclareLaunchArgument(
        "objects_info",
        default_value=default_objects_info,
        description="Absolute path to the objects.yaml configuration file.",
    )

    yolo_classes_info_arg = DeclareLaunchArgument(
        "yolo_classes_info",
        default_value=default_yolo_classes_info,
        description="Absolute path to the YOLO class matching YAML file.",
    )

    yolo_weights_arg = DeclareLaunchArgument(
        "yolo_weights",
        default_value=default_yolo_weights,
        description="Absolute path to the target YOLO .onnx network model weights.",
    )

    # 3. Create active Launch Configurations to bind evaluation run-time context
    objects_info_config = LaunchConfiguration("objects_info")
    yolo_classes_info_config = LaunchConfiguration("yolo_classes_info")
    yolo_weights_config = LaunchConfiguration("yolo_weights")

    # 4. Construct container with evaluation variables bound to parameters
    container = ComposableNodeContainer(
        name="MMOR_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="mir_object_recognition",
                plugin="perception_namespace::MultiModalObjectRecognitionROS",
                name=node_name,
                remappings=[
                    ("input_image_topic", "/camera/color/image_raw"),
                    ("input_cloud_topic", "/camera/depth/color/points"),
                ],
                parameters=[
                    {"objects_info": objects_info_config},
                    {"yolo_classes_info": yolo_classes_info_config},
                    {"yolo_weights": yolo_weights_config},
                ],
            )
        ],
        parameters=[default_scene_seg_constraints],
        output="screen",
        prefix=["xterm -e gdb -ex run --args"],
    )

    # transition mmor node to shutdown before SIGINT
    shutdown_event = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_node_name(node_name=node_name),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVE_SHUTDOWN,
                    )
                ),
                LogInfo(msg="[mmor_launch] mmor node is exiting."),
            ],
        )
    )

    # 5. Populate the launch description object sequentially
    ld.add_action(objects_info_arg)
    ld.add_action(yolo_classes_info_arg)
    ld.add_action(yolo_weights_arg)
    ld.add_action(shutdown_event)
    ld.add_action(container)

    return ld
