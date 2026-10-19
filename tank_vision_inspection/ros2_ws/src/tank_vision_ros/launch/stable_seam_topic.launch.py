from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("tank_vision_ros")
    return LaunchDescription([
        DeclareLaunchArgument("image_topic", default_value="/camera/front/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/front/camera_info"),
        DeclareLaunchArgument("config_file", default_value=f"{share}/config/s100p_stable_seam_4096x3072.yaml"),
        DeclareLaunchArgument("reference_file", default_value=f"{share}/config/seam_width_reference_s100p_v1.json"),
        DeclareLaunchArgument("publish_debug_image", default_value="false"),
        DeclareLaunchArgument("roi_validation_recording_enabled", default_value="false"),
        DeclareLaunchArgument("roi_validation_output_root", default_value="/media/mcap_data/runs/roi_validation"),
        DeclareLaunchArgument("roi_validation_queue_size", default_value="8"),
        DeclareLaunchArgument("roi_validation_max_duration_sec", default_value="0.0"),
        DeclareLaunchArgument("geometry_provenance_sampling_enabled", default_value="false"),
        DeclareLaunchArgument("geometry_provenance_output_root", default_value="/media/mcap_data/runs/geometry_provenance"),
        DeclareLaunchArgument("geometry_provenance_queue_size", default_value="8"),
        DeclareLaunchArgument("geometry_provenance_sample_period_sec", default_value="5.0"),
        DeclareLaunchArgument("geometry_provenance_max_events", default_value="30"),
        Node(
            package="tank_vision_ros", executable="stable_seam_node", name="stable_seam_node", output="screen",
            parameters=[{
                "input_mode": "ros_topic", "image_topic": LaunchConfiguration("image_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "config_file": LaunchConfiguration("config_file"),
                "reference_file": LaunchConfiguration("reference_file"),
                "publish_debug_image": LaunchConfiguration("publish_debug_image"),
                "roi_validation_recording_enabled": LaunchConfiguration("roi_validation_recording_enabled"),
                "roi_validation_output_root": LaunchConfiguration("roi_validation_output_root"),
                "roi_validation_queue_size": LaunchConfiguration("roi_validation_queue_size"),
                "roi_validation_max_duration_sec": LaunchConfiguration("roi_validation_max_duration_sec"),
                "geometry_provenance_sampling_enabled": LaunchConfiguration("geometry_provenance_sampling_enabled"),
                "geometry_provenance_output_root": LaunchConfiguration("geometry_provenance_output_root"),
                "geometry_provenance_queue_size": LaunchConfiguration("geometry_provenance_queue_size"),
                "geometry_provenance_sample_period_sec": LaunchConfiguration("geometry_provenance_sample_period_sec"),
                "geometry_provenance_max_events": LaunchConfiguration("geometry_provenance_max_events"),
            }],
        )
    ])
