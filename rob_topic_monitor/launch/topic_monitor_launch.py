# Copyright 2024 Robert Bosch GmbH and its subsidiaries
#
# All rights reserved, also regarding any disposal, exploitation, reproduction,
# editing, distribution, as well as in the event of applications for industrial
# property rights.

from launch import LaunchDescription
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

DIAG_NAMESPACE = "/diagnostics"
FREQ_MONITOR_NAME = "topic_frequency_monitor"
AGE_MONITOR_NAME = "message_age_monitor"


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
                "use_sim_time", default_value="False", description="Whether to use sim_time"
            ),
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        DeclareLaunchArgument(
            "namespace", default_value="", description="Top-level namespace"
        ),
        GroupAction(
            actions=[
                PushRosNamespace(namespace=DIAG_NAMESPACE),
                Node(
                    package="rob_topic_monitor",
                    executable="topic_monitor",
                    name=FREQ_MONITOR_NAME,
                    parameters=[
                        {'use_sim_time': LaunchConfiguration('use_sim_time'),
                            'monitor_configured_only': True},
                        PathJoinSubstitution([
                            FindPackageShare("rob_topic_monitor"),
                            "config",
                            "topic_check.yaml"])
                        ]
                ),
                Node(
                    package="rob_topic_monitor",
                    executable="header_topic_monitor",
                    name=AGE_MONITOR_NAME,
                    parameters=[
                        {'use_sim_time': LaunchConfiguration('use_sim_time'),
                            'monitor_configured_only': False},
                        PathJoinSubstitution([
                            FindPackageShare("rob_topic_monitor"),
                            "config",
                            "topic_check.yaml"])
                        ]
                ),
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    namespace="",
                    name="lifecycle_manager_module_diag",
                    output="both",
                    parameters=[
                        {'use_sim_time': LaunchConfiguration('use_sim_time'),
                            "autostart": True,
                            "node_names": [
                            [DIAG_NAMESPACE, "/", FREQ_MONITOR_NAME],
                            [DIAG_NAMESPACE, "/", AGE_MONITOR_NAME]
                            ],
                            'bond_timeout': 0.0},
                    ],
                ),
            ]
        )
    ])
