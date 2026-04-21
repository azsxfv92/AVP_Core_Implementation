from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="avp_core_implementation",
            executable="trt_infer_node",
            name="trt_image_infer_node",
            output="screen",
            parameters=[
                {
                    "engine_path": "models/trt/yolov5n_fp16.engine",
                    "image_topic": "/avp/camera/front",
                    "overlay_topic": "/avp/infer/overlay",
                }
            ]
            )
    ])