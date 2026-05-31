from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    preprocess_backend = LaunchConfiguration("preprocess_backend")
    csv_path = LaunchConfiguration("csv_path")
    engine_path = LaunchConfiguration("engine_path")
    image_topic = LaunchConfiguration("image_topic")
    overlay_topic = LaunchConfiguration("overlay_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "preprocess_backend",
            default_value="cpu",
            description="Preprocess backend: cpu or cuda"
        ),
        DeclareLaunchArgument(
            "csv_path",
            default_value="results/week11/stage_metrics/stage_times.csv",
            description="CSV path for stage timing metrics"
        ),
        DeclareLaunchArgument(
            "engine_path",
            default_value="models/trt/yolov5n_fp16.engine",
            description="TensorRT engine path"
        ),
        DeclareLaunchArgument(
            "image_topic",
            default_value="/avp/camera/front",
            description="Input image topic"
        ),
        DeclareLaunchArgument(
            "overlay_topic",
            default_value="/avp/infer/overlay",
            description="Overlay output topic"
        ),
        DeclareLaunchArgument(
            "enable_encode",
            default_value="false",
            description="Enable GStreamer video encode/save stage",
        ),
        DeclareLaunchArgument(
            "output_video_path",
            default_value="results/week12/output/week12_overlay.mp4",
            description="Output video path for encoded overlay result",
        ),
        DeclareLaunchArgument(
            "encode_fps",
            default_value="30",
            description="Target FPS for video encoder",
        ),
        DeclareLaunchArgument(
            "encode_width",
            default_value="1280",
            description="Output video width for encoder",
        ),
        DeclareLaunchArgument(
            "encode_height",
            default_value="720",
            description="Output video height for encoder",
        ),
        DeclareLaunchArgument(
            "encode_queue_size",
            default_value="4",
            description="Bounded queue size for encode/save stage",
        ),

        Node(
            package="avp_core_implementation",
            executable="trt_infer_node",
            name="trt_image_infer_node",
            output="screen",
            parameters=[
                {
                    "engine_path": engine_path,
                    "image_topic": image_topic,
                    "overlay_topic": overlay_topic,
                    "csv_path": csv_path,
                    "preprocess_backend": preprocess_backend,
                    "enable_encode": LaunchConfiguration("enable_encode"),
                    "output_video_path": LaunchConfiguration("output_video_path"),
                    "encode_fps": LaunchConfiguration("encode_fps"),
                    "encode_width": LaunchConfiguration("encode_width"),
                    "encode_height": LaunchConfiguration("encode_height"),
                    "encode_queue_size": LaunchConfiguration("encode_queue_size"),
                }
            ]
            )
    ])