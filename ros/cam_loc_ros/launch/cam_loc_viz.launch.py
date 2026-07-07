from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("cam_loc_ros")
    rviz_config = os.path.join(pkg_share, "rviz", "cam_loc.rviz")

    kitti_root = LaunchConfiguration("kitti_root")
    perception_mode = LaunchConfiguration("perception_mode")
    use_gt_plane = LaunchConfiguration("use_gt_plane")
    sequence = LaunchConfiguration("sequence")
    playback_hz = LaunchConfiguration("playback_hz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "kitti_root",
                default_value=os.path.join(
                    os.path.dirname(pkg_share), "..", "..", "..", "data", "smoke_kitti"
                ),
            ),
            DeclareLaunchArgument("perception_mode", default_value="oracle"),
            DeclareLaunchArgument("use_gt_plane", default_value="true"),
            DeclareLaunchArgument("sequence", default_value="0"),
            DeclareLaunchArgument("playback_hz", default_value="5.0"),
            Node(
                package="cam_loc_ros",
                executable="cam_loc_viz_node",
                name="cam_loc_viz",
                namespace="cam_loc",
                output="screen",
                parameters=[
                    {
                        "kitti_root": kitti_root,
                        "perception_mode": perception_mode,
                        "use_gt_plane": use_gt_plane,
                        "sequence": sequence,
                        "playback_hz": playback_hz,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
