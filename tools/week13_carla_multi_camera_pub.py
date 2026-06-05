#!/usr/bin/env python3
import sys
from pathlib import Path

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


try:
    import carla
except ImportError as e:
    print(f"[ERROR] import carla failed: {e}")
    sys.exit(1)


class CarlaImagePublisher(Node):
    def __init__(self):
        
        # change Relialility to BestEffort
        super().__init__("week13_carla_multi_camera_pub")
        camera_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub_front = self.create_publisher(CompressedImage, "/avp/camera/front/compressed", camera_qos)
        self.pub_rear = self.create_publisher(CompressedImage, "/avp/camera/rear/compressed", camera_qos)

        self.output_dir = Path("results/week13/multistream")
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.host = "127.0.0.1"
        self.port = 2000
        self.timeout_sec = 5.0

        self.vehicle = None
        self.front_camera = None
        self.rear_camera = None
        self.received_count = 0
        self.saved_first_png = {
            "front": 0,
            "rear": 0,
        }

        self.client = None
        self.world = None
        self.blueprint_library = None
        self.jpeg_quality = 80
        self.received_count = {
            "front": 0,
            "rear": 0,
        }

    def destroy_existing_hero(self):
        actors = self.world.get_actors()

        hero_vehicle_ids = []
        attached_sensor_ids = []

        for actor in actors:
            if actor.type_id.startswith("vehicle."):
                if actor.attributes.get("role_name") == "hero":
                    hero_vehicle_ids.append(actor.id)
        
        for actor in actors:
            parent= actor.parent
            if parent is not None and parent.id in hero_vehicle_ids:
                attached_sensor_ids.append(actor.id)
        
        for actor_id in attached_sensor_ids:
            actor = self.world.get_actor(actor_id)
            if actor is not None:
                self.get_logger().info(
                    f"[CLEANUP-BEFORE] destroy sensor id={actor.id}, type={actor.type_id}"
                )
                actor.destroy()
        
        for actor_id in hero_vehicle_ids:
            actor = self.world.get_actor(actor_id)
            if actor is not None:
                self.get_logger().info(
                    f"[CLEANUP-BEFORE] destroy vehicle id={actor.id}, type={actor.type_id}"
                )
                actor.destroy()

    def setup_carla(self):
        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(self.timeout_sec)
        
        self.world = self.client.get_world()
        self.blueprint_library = self.world.get_blueprint_library()

        self.destroy_existing_hero()

        vehicle_bp = self.blueprint_library.find("vehicle.tesla.model3")
        vehicle_bp.set_attribute("role_name", "hero")

        spawn_points = self.world.get_map().get_spawn_points()
        if not spawn_points:
            raise RuntimeError("No spawn points found")

        self.vehicle = None
        for sp in spawn_points:
            self.vehicle = self.world.try_spawn_actor(vehicle_bp, sp)
            if self.vehicle is not None:
                break
        
        if self.vehicle is None:
            raise RuntimeError("No available spawn point found for hero vehicle")

        camera_bp = self.blueprint_library.find("sensor.camera.rgb")
        camera_bp.set_attribute("image_size_x", "640")
        camera_bp.set_attribute("image_size_y", "360")
        camera_bp.set_attribute("fov", "90")
        camera_bp.set_attribute("sensor_tick", "0.1")

        front_camera_transform = carla.Transform(
            carla.Location(x=1.5, z=2.4),
            carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0)
        )
        rear_camera_transform = carla.Transform(
            carla.Location(x=-1.5, z=2.4),
            carla.Rotation(pitch=0.0, yaw=180.0, roll=0.0)
        )

        self.front_camera = self.world.spawn_actor(
            camera_bp,
            front_camera_transform,
            attach_to=self.vehicle
        )

        self.rear_camera = self.world.spawn_actor(
            camera_bp,
            rear_camera_transform,
            attach_to=self.vehicle
        )

        self.get_logger().info(
            f"front camera created: id={self.front_camera.id}, tick={self.front_camera.attributes.get('sensor_tick')}"
        )

        self.get_logger().info(
            f"rear camera created: id={self.rear_camera.id}, tick={self.rear_camera.attributes.get('sensor_tick')}"
        )

        self.front_camera.listen(lambda image: self.on_image(image, 'front'))
        self.rear_camera.listen(lambda image: self.on_image(image, 'rear'))

    def on_image(self, image, stream_id: str):
        self.received_count[stream_id] += 1

        raw = np.frombuffer(image.raw_data, dtype=np.uint8)
        raw = raw.reshape((image.height, image.width, 4))
        
        bgr = raw[:, :, :3]

        if not self.saved_first_png[stream_id]:
            png_path = self.output_dir / f"frame_{image.frame:06d}.png"
            cv2.imwrite(str(png_path), bgr)
            self.saved_first_png[stream_id] = True
            self.get_logger().info(f"first PNG saved: {png_path}")

        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality),]
        ok, encoded = cv2.imencode(".jpg", bgr, encode_params)
        if not ok:
            self.get_logger().error("JPEG encode failed")
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"carla_{stream_id}_camera"
        msg.format = "jpeg"
        msg.data = encoded.tobytes()

        if stream_id == 'front':
            self.pub_front.publish(msg)
        elif stream_id == 'rear':
            self.pub_rear.publish(msg)

        if (self.received_count[stream_id] % 30) == 0:
            raw_bytes = image.width * image.height * 3
            compressed_bytes = len(msg.data)
            compression_ratio = raw_bytes / compressed_bytes if compressed_bytes > 0 else 0.0

            self.get_logger().info(
                f"[PUB-COMPRESSED] frame={image.frame}, "
                f"size={image.width}x{image.height}, "
                f"raw={raw_bytes / 1024.0:.1f}KB, "
                f"compressed={compressed_bytes / 1024.0:.1f}KB, "
                f"ratio={compression_ratio:.2f}, "
                f"published_count={self.received_count[stream_id]}"
            )

    def cleanup(self):
        if self.front_camera is not None:
            self.front_camera.stop()
            self.front_camera.destroy()
            self.get_logger().info("front_camera destroyed")
            self.front_camera = None

        if self.rear_camera is not None:
            self.rear_camera.stop()
            self.rear_camera.destroy()
            self.get_logger().info("rear_camera destroyed")
            self.rear_camera = None
        
        if self.vehicle is not None:
            self.vehicle.destroy()
            self.get_logger().info("vehicle destroyed")
            self.vehicle = None

def main():
    rclpy.init()

    node = CarlaImagePublisher()

    try:
        node.setup_carla()
        node.get_logger().info("CARLA image publisher started")
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, shutting down")
    except Exception as e:
        node.get_logger().error(f"Exception: {e}")
    
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()






