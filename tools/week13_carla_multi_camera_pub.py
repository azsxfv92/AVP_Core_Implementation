#!/usr/bin/env python3
import random
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
        self.timeout_sec = 20.0

        # --- Week16 Day3: 트래픽 생성 파라미터 ---
        self.declare_parameter("town", "")            # 예: Town10HD, 빈 문자열이면 현재 맵 유지
        self.declare_parameter("num_vehicles", 80)
        self.declare_parameter("num_walkers", 40)
        self.declare_parameter("hero_autopilot", True)
        self.declare_parameter("tm_port", 8000)
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 360)

        self.town = self.get_parameter("town").value
        self.num_vehicles = self.get_parameter("num_vehicles").value
        self.num_walkers = self.get_parameter("num_walkers").value
        self.hero_autopilot = self.get_parameter("hero_autopilot").value
        self.tm_port = self.get_parameter("tm_port").value
        self.image_width = self.get_parameter("image_width").value
        self.image_height = self.get_parameter("image_height").value

        self.traffic_manager = None
        self.npc_vehicle_ids = []
        self.walker_ids = []
        self.controller_ids = []

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

    def destroy_existing_traffic(self):
        """이전 실행이 남긴 NPC 를 정리한다. 재실행할수록 액터가 쌓이는 것을 막는다."""
        actors = self.world.get_actors()
        stale = []

        for actor in actors:
            if actor.type_id.startswith("controller.ai.walker"):
                actor.stop()
                stale.append(actor.id)
            elif actor.type_id.startswith("walker.pedestrian."):
                stale.append(actor.id)
            elif actor.type_id.startswith("vehicle."):
                if actor.attributes.get("role_name") == "npc":
                    stale.append(actor.id)

        if stale:
            self.client.apply_batch([carla.command.DestroyActor(x) for x in stale])
            self.get_logger().info(f"[CLEANUP-BEFORE] destroyed {len(stale)} stale traffic actors")

    def spawn_traffic(self):
        """generate_traffic.py 를 대신한다. CARLA_ROOT 나 PythonAPI/examples 가 필요 없다."""
        if self.num_vehicles <= 0 and self.num_walkers <= 0:
            self.get_logger().info("traffic spawn skipped (num_vehicles=0, num_walkers=0)")
            return

        self.traffic_manager = self.client.get_trafficmanager(self.tm_port)
        self.traffic_manager.set_synchronous_mode(False)
        self.traffic_manager.set_global_distance_to_leading_vehicle(2.0)

        # --- NPC 차량 ---
        vehicle_bps = [
            bp for bp in self.blueprint_library.filter("vehicle.*")
            if int(bp.get_attribute("number_of_wheels")) == 4
        ]
        spawn_points = self.world.get_map().get_spawn_points()
        random.shuffle(spawn_points)

        n_vehicles = min(self.num_vehicles, len(spawn_points))
        if n_vehicles < self.num_vehicles:
            self.get_logger().warn(
                f"spawn points available={len(spawn_points)}, requested={self.num_vehicles}"
            )

        batch = []
        for sp in spawn_points[:n_vehicles]:
            bp = random.choice(vehicle_bps)
            bp.set_attribute("role_name", "npc")
            if bp.has_attribute("color"):
                bp.set_attribute("color", random.choice(bp.get_attribute("color").recommended_values))
            batch.append(
                carla.command.SpawnActor(bp, sp).then(
                    carla.command.SetAutopilot(carla.command.FutureActor, True, self.tm_port)
                )
            )

        for response in self.client.apply_batch_sync(batch, False):
            if response.error:
                self.get_logger().warn(f"vehicle spawn failed: {response.error}")
            else:
                self.npc_vehicle_ids.append(response.actor_id)

        self.get_logger().info(f"NPC vehicles spawned: {len(self.npc_vehicle_ids)}")

        # --- 보행자 ---
        if self.num_walkers > 0:
            self.world.set_pedestrians_cross_factor(0.5)
            walker_bps = self.blueprint_library.filter("walker.pedestrian.*")

            walker_batch = []
            for _ in range(self.num_walkers):
                loc = self.world.get_random_location_from_navigation()
                if loc is None:
                    continue
                bp = random.choice(walker_bps)
                if bp.has_attribute("is_invincible"):
                    bp.set_attribute("is_invincible", "false")
                walker_batch.append(
                    carla.command.SpawnActor(bp, carla.Transform(loc))
                )

            for response in self.client.apply_batch_sync(walker_batch, True):
                if response.error:
                    self.get_logger().warn(f"walker spawn failed: {response.error}")
                else:
                    self.walker_ids.append(response.actor_id)

            controller_bp = self.blueprint_library.find("controller.ai.walker")
            controller_batch = [
                carla.command.SpawnActor(controller_bp, carla.Transform(), walker_id)
                for walker_id in self.walker_ids
            ]
            for response in self.client.apply_batch_sync(controller_batch, True):
                if response.error:
                    self.get_logger().warn(f"walker controller spawn failed: {response.error}")
                else:
                    self.controller_ids.append(response.actor_id)

            # 컨트롤러가 서버에 등록될 때까지 한 틱 기다린 뒤 걷게 한다
            self.world.wait_for_tick()
            for controller in self.world.get_actors(self.controller_ids):
                controller.start()
                target = self.world.get_random_location_from_navigation()
                if target is not None:
                    controller.go_to_location(target)
                controller.set_max_speed(1.4)

            self.get_logger().info(f"walkers spawned: {len(self.walker_ids)}")

    def setup_carla(self):
        self.client = carla.Client(self.host, self.port)
        self.client.set_timeout(self.timeout_sec)

        if self.town:
            self.get_logger().info(f"loading map: {self.town} (기존 액터가 모두 삭제된다)")
            self.world = self.client.load_world(self.town)
        else:
            self.world = self.client.get_world()

        # generate_traffic.py 가 동기 모드로 남겨둔 월드를 되돌린다
        settings = self.world.get_settings()
        if settings.synchronous_mode:
            settings.synchronous_mode = False
            settings.fixed_delta_seconds = None
            self.world.apply_settings(settings)
            self.get_logger().warn("world was in synchronous mode, restored to async")

        self.blueprint_library = self.world.get_blueprint_library()

        self.destroy_existing_hero()
        self.destroy_existing_traffic()
        self.spawn_traffic()

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

        # --- Week16 Day3: ego 주행 ---
        if self.hero_autopilot:
            if self.traffic_manager is None:
                self.traffic_manager = self.client.get_trafficmanager(self.tm_port)
            self.vehicle.set_autopilot(True, self.tm_port)
            self.traffic_manager.vehicle_percentage_speed_difference(self.vehicle, 30.0)
            self.get_logger().info(f"hero autopilot enabled (tm_port={self.tm_port})")

        camera_bp = self.blueprint_library.find("sensor.camera.rgb")
        camera_bp.set_attribute("image_size_x", str(self.image_width))
        camera_bp.set_attribute("image_size_y", str(self.image_height))
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

        # --- Week16 Day3: 스폰한 트래픽 정리 ---
        if self.client is None:
            return

        if self.controller_ids:
            for controller in self.world.get_actors(self.controller_ids):
                controller.stop()
            self.client.apply_batch([carla.command.DestroyActor(x) for x in self.controller_ids])
            self.get_logger().info(f"{len(self.controller_ids)} walker controllers destroyed")
            self.controller_ids = []

        if self.walker_ids:
            self.client.apply_batch([carla.command.DestroyActor(x) for x in self.walker_ids])
            self.get_logger().info(f"{len(self.walker_ids)} walkers destroyed")
            self.walker_ids = []

        if self.npc_vehicle_ids:
            self.client.apply_batch([carla.command.DestroyActor(x) for x in self.npc_vehicle_ids])
            self.get_logger().info(f"{len(self.npc_vehicle_ids)} NPC vehicles destroyed")
            self.npc_vehicle_ids = []

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






