"""Bring up three Triton cameras for synchronized (PTP action-command) capture.

A separate sync_trigger node broadcasts the scheduled action commands, so the
cameras are interchangeable and the trigger has no camera to take down with it.

Synchronized recording (default):
    ros2 launch arena_camera_node sync_capture.launch.py \
        serial_0:=<s0> serial_1:=<s1> serial_2:=<s2>

Free-run baseline (no trigger sync, PTP still on so timestamps stay comparable):
    ros2 launch arena_camera_node sync_capture.launch.py \
        serial_0:=<s0> serial_1:=<s1> serial_2:=<s2> trigger_source:=continuous
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("serial_0", default_value=""),
        DeclareLaunchArgument("serial_1", default_value=""),
        DeclareLaunchArgument("serial_2", default_value=""),
        DeclareLaunchArgument("trigger_source", default_value="action",
                              description="action (synced) or continuous (baseline)"),
        DeclareLaunchArgument("trigger_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("exposure_time", default_value="1000.0",
                              description="microseconds; must be equal on all cameras"),
        DeclareLaunchArgument("pixelformat", default_value="mono8"),
    ]

    trigger_source = LaunchConfiguration("trigger_source")

    def camera(index):
        return Node(
            package="arena_camera_node",
            executable="arena_camera_node",
            name=f"cam{index}",
            parameters=[{
                "serial": LaunchConfiguration(f"serial_{index}"),
                "camera_type": "high_speed",
                "pixelformat": LaunchConfiguration("pixelformat"),
                "exposure_time": LaunchConfiguration("exposure_time"),
                "trigger_source": trigger_source,
                "ptp_enable": True,
                "topic": f"/cam{index}/images",
            }],
            output="screen",
        )

    # The trigger broadcaster runs only when actually synchronizing.
    sync_trigger = Node(
        package="arena_camera_node",
        executable="sync_trigger",
        name="sync_trigger",
        parameters=[{"trigger_rate_hz": LaunchConfiguration("trigger_rate_hz")}],
        output="screen",
        condition=IfCondition(PythonExpression(["'", trigger_source, "' == 'action'"])),
    )

    return LaunchDescription(args + [camera(0), camera(1), camera(2), sync_trigger])
