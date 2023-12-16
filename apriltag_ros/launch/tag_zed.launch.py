import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

image_topic_ = LaunchConfiguration("image_topic", default="image_rect_color")
camera_name = LaunchConfiguration("camera_name", default="/zed_" + os.environ.get("cam_loc") + "/zed_node_" + os.environ.get("cam_loc")+"/left")

image_topic = [camera_name, "/", image_topic_]
# info_topic = [camera_name, "/camera_info"]
config = os.path.join(
    get_package_share_directory("apriltag_ros"), "cfg", "tags_36h11_hewithall_zed.yaml"
)


def generate_launch_description():

    composable_node = ComposableNode(
        name="apriltag",
        package="apriltag_ros",
        plugin="AprilTagNode",
        parameters=[config],
        remappings=[("/image", image_topic),
                    ("/apriltag_detections", "/apriltag_detections_zed_" + os.environ.get("cam_loc"))]
    )

    container = ComposableNodeContainer(
        name="tag_container_zed",
        namespace="apriltag_zed",
        package="rclcpp_components",
        executable="component_container",
        prefix="gdbserver localhost:8009",
        composable_node_descriptions=[composable_node],
        output="screen",
    )

    return launch.LaunchDescription([container])
