#!/usr/bin/env python3
import sys
import time
from pathlib import Path

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    import carla
except ImportError as e:
    print(f"[ERROR] import carla failed: {e}")
    sys.exit(1)


class CarlaImagePublisher(Node):
    def __init__(self):
        super().__init__("week10_carla_image_pub")

        self.pub_ = self.create_publisher(Image, "/avp/camera/front", 10)

        self.output_dir = Path("results/week10/carla_image_pub")
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.host = "127.0.0.1"
        self.port = 2000
        self.timeout_sec = 5.0

        self.vehicle = None
        self.camera = None
        self.received_count = 0
        self.saved_first_png = False

        self.client = None
        self.world = None
        self.blueprint_library = None

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
            actor = self.world.get_actors(actor_id)
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
        camera_bp.set_attribute("image_size_x", "1280")
        camera_bp.set_attribute("image_size_y", "720")
        camera_bp.set_attribute("fov", "90")
        camera_bp.set_attribute("sensor_tick", "0.1")

        camera_transform = carla.Transform(
            carla.Location(x=1.5, z=2.4)
        )

        self.camera = self.world.spawn_actor(
            camera_bp,
            camera_transform,
            attach_to=self.vehicle
        )

        self.get_logger().info(
            f"camera created: id={self.camera.id}, tick={self.camera.attributes.get('sensor_tick')}"
        )

        self.camera.listen(self.on_image)

    def on_image(self, image):
        self.received_count += 1

        raw = np.frombuffer(image.raw_data, dtype=np.uint8)
        raw = raw.reshape((image.height, image.width, 4))
        
        bgr = raw[:, :, :3]

        if not self.saved_first_png:
            png_path = self.output_dir / f"frame_{image.frame:06d}.png"
            cv2.imwrite(str(png_path), bgr)
            self.saved_first_png = True
            self.get_logger().info(f"first PNG saved: {png_path}")
        
        msg = Image()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "carla_front_camera"

        msg.height = image.height
        msg.width = image.width
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = image.width * 3
        msg.data = bgr.tobytes()

        self.pub_.publish(msg)

        if (self.received_count % 30) == 0:
            self.get_logger().info(
                f"[PUB] frame={image.frame}, size={image.width}x{image.height}, published_count={self.received_count}"
            )

    def cleanup(self):
        if self.camera is not None:
            self.camera.stop()
            self.camera.destroy()
            self.get_logger().info("camera destroyed")
            self.camera = None
        
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






