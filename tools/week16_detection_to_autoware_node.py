import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from vision_msgs.msg import Detection2DArray
from autoware_perception_msgs.msg import (
    DetectedObjects,
    DetectedObject,
    ObjectClassification,
    Shape,
)

# COCO class id -> (Autoware label, length, width, height)
COCO_TO_AUTOWARE = {
    0: (ObjectClassification.PEDESTRIAN, 0.6, 0.6, 1.7),
    1: (ObjectClassification.BICYCLE,    1.8, 0.6, 1.5),
    2: (ObjectClassification.CAR,        4.5, 1.9, 1.5),
    3: (ObjectClassification.MOTORCYCLE, 2.2, 0.8, 1.5),
    5: (ObjectClassification.BUS,       12.0, 2.6, 3.2),
    7: (ObjectClassification.TRUCK,      8.0, 2.5, 3.0),
}
DEFAULT_ENTRY = (ObjectClassification.UNKNOWN, 1.0, 1.0, 1.0)


class DetectionToAutoware(Node):
    def __init__(self):
        super().__init__("detection_to_autoware_node")

        self.declare_parameter("input_topic", "/avp/detections")
        self.declare_parameter("output_topic",
                               "/perception/object_recognition/detection/objects")
        self.declare_parameter("output_frame", "base_link")
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 360)
        self.declare_parameter("fov_deg", 90.0)
        self.declare_parameter("camera_height", 2.4)
        self.declare_parameter("camera_offset_x", 1.5)
        self.declare_parameter("max_range", 80.0)

        self.in_topic  = self.get_parameter("input_topic").value
        self.out_topic = self.get_parameter("output_topic").value
        self.frame_id  = self.get_parameter("output_frame").value
        self.img_w     = self.get_parameter("image_width").value
        self.img_h     = self.get_parameter("image_height").value
        self.cam_h     = self.get_parameter("camera_height").value
        self.cam_x     = self.get_parameter("camera_offset_x").value
        self.max_range = self.get_parameter("max_range").value

        fov_rad = math.radians(self.get_parameter("fov_deg").value)
        self.fx = (self.img_w / 2.0) / math.tan(fov_rad / 2.0)
        self.fy = self.fx
        self.cx = self.img_w / 2.0
        self.cy = self.img_h / 2.0

        # it should match with Autoware multi_object_tracker
        aw_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(DetectedObjects, self.out_topic, aw_qos)

        # it should match with QoS of trt_infer_node publisher
        in_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.sub = self.create_subscription(
            Detection2DArray, self.in_topic, self.on_detections, in_qos)

        self.frame_count = 0
        self.get_logger().info(
            f"bridge up: {self.in_topic} -> {self.out_topic} "
            f"(fx={self.fx:.1f}, cy={self.cy:.1f}, frame={self.frame_id})"
        )

    def project_to_ground(self, u, v_bottom):
        """image coordinate -> base_link 3D position."""
        denom = v_bottom - self.cy
        if denom <= 1.0:                 
            return None
        z_fwd = self.fy * self.cam_h / denom
        if z_fwd > self.max_range:
            return None
        x_lat = (u - self.cx) * z_fwd / self.fx
        return (z_fwd + self.cam_x, -x_lat, 0.0)

    def on_detections(self, msg: Detection2DArray):
        out = DetectedObjects()
        out.header.stamp = msg.header.stamp       
        out.header.frame_id = self.frame_id

        dropped = 0
        for det in msg.detections:
            if not det.results:
                continue

            hyp = det.results[0]
            
            if hasattr(hyp, "hypothesis"):
                class_id_str = hyp.hypothesis.class_id
                score = float(hyp.hypothesis.score)
            else:
                class_id_str = str(hyp.id)
                score = float(hyp.score)

            try:
                coco_id = int(class_id_str)
            except ValueError:
                coco_id = -1

            cx_px = float(det.bbox.center.position.x) \
                if hasattr(det.bbox.center, "position") else float(det.bbox.center.x)
            cy_px = float(det.bbox.center.position.y) \
                if hasattr(det.bbox.center, "position") else float(det.bbox.center.y)
            size_x = float(det.bbox.size_x)
            size_y = float(det.bbox.size_y)

            v_bottom = cy_px + size_y / 2.0
            pos = self.project_to_ground(cx_px, v_bottom)
            if pos is None:
                dropped += 1
                continue

            label, dim_l, dim_w, dim_h = COCO_TO_AUTOWARE.get(coco_id, DEFAULT_ENTRY)

            obj = DetectedObject()
            obj.existence_probability = score

            cls = ObjectClassification()
            cls.label = label
            cls.probability = score
            obj.classification.append(cls)

            obj.kinematics.pose_with_covariance.pose.position.x = pos[0]
            obj.kinematics.pose_with_covariance.pose.position.y = pos[1]
            obj.kinematics.pose_with_covariance.pose.position.z = dim_h / 2.0
            obj.kinematics.pose_with_covariance.pose.orientation.w = 1.0
            obj.kinematics.has_position_covariance = False
            obj.kinematics.orientation_availability = 0   # UNAVAILABLE
            obj.kinematics.has_twist = False
            obj.kinematics.has_twist_covariance = False

            obj.shape.type = Shape.BOUNDING_BOX
            obj.shape.dimensions.x = dim_l
            obj.shape.dimensions.y = dim_w
            obj.shape.dimensions.z = dim_h

            out.objects.append(obj)

        self.pub.publish(out)

        self.frame_count += 1
        if self.frame_count % 20 == 0:
            self.get_logger().info(
                f"in={len(msg.detections)} out={len(out.objects)} dropped={dropped}"
            )


def main():
    rclpy.init()
    node = DetectionToAutoware()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()