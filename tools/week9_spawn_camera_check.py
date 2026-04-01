#!/usr/bin/env python3
import sys
import time
from pathlib import Path

import numpy as np
import cv2

try:
    import carla
except ImportError as e:
    print(f"[ERROR] import carla failed: {e}")
    sys.exit(1)

def destroy_existing_hero(world):
    actors = world.get_actors()

    hero_vehicle_ids = []
    attached_sensor_ids = []

    for actor in actors:
        if actor.type_id.startswith("vehicle."):
            if actor.attributes.get("role_name") == "hero":
                hero_vehicle_ids.append(actor.id)

    for actor in actors:
        parent = actor.parent
        if parent is not None and parent.id in hero_vehicle_ids:
            attached_sensor_ids.append(actor.id)

    for actor_id in attached_sensor_ids:
        actor = world.get_actor(actor_id)
        if actor is not None:
            print(f"[CLEANUP-BEFORE] destroy attached sensor id={actor.id}, type={actor.type_id}")
            actor.destroy()

    for actor_id in hero_vehicle_ids:
        actor = world.get_actor(actor_id)
        if actor is not None:
            print(f"[CLEANUP-BEFORE] destroy hero vehicle id={actor.id}, type={actor.type_id}")
            actor.destroy()

def main():
    host = "127.0.0.1"
    port = 2000
    timeout_sec = 5.0

    output_dir = Path("results/week9/camera_check")
    output_dir.mkdir(parents=True, exist_ok=True)

    vehicle = None
    camera = None
    received_frames = []
    saved_first_png = False

    try:
        client = carla.Client(host, port)
        client.set_timeout(timeout_sec)

        world = client.get_world()
        blueprint_library = world.get_blueprint_library()

        destroy_existing_hero(world)

        vehicle_bp = blueprint_library.find("vehicle.tesla.model3")
        vehicle_bp.set_attribute("role_name", "hero")
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            raise RuntimeError("No spawn points found in current map")

        # spawn_point = spawn_points[0]
        vehicle = None
        spwan_point = None
        for sp in spawn_points:
            vehicle = world.try_spawn_actor(vehicle_bp, sp)
            if vehicle is not None:
                spawn_point = sp
                break
        if vehicle is None:
            raise RuntimeError("No available spawn point found for hero vehicle")

        # vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        # print(f"[OK] vehicle spawned: id={vehicle.id}, type={vehicle.type_id}")


        camera_bp = blueprint_library.find("sensor.camera.rgb")

        camera_bp.set_attribute("image_size_x", "1280")
        camera_bp.set_attribute("image_size_y", "720")
        camera_bp.set_attribute("fov", "90")
        camera_bp.set_attribute("sensor_tick", "0.1")  # 10 Hz

        camera_transform = carla.Transform(
            carla.Location(x=1.5, z=2.4)
        )

        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
        print("[DEBUG] camera sensor_tick =", camera.attributes.get("sensor_tick"))
        print("[DEBUG] camera image_size_x =", camera.attributes.get("image_size_x"))
        print("[DEBUG] camera image_size_y =", camera.attributes.get("image_size_y"))
        print("[DEBUG] vehicle role_name =", vehicle.attributes.get("role_name"))

        # camera sensor callback function
        def on_image(image):
            nonlocal saved_first_png
            received_frames.append(image)

            meta_path = output_dir / f"frame_{image.frame:06d}.txt"
            with open(meta_path, "w", encoding="utf-8") as f:
                f.write(f"frame={image.frame}\n")
                f.write(f"timestamp={image.timestamp}\n")
                f.write(f"width={image.width}\n")
                f.write(f"height={image.height}\n")
                f.write(f"raw_data_size={len(image.raw_data)}\n")
            
            if not saved_first_png:
                raw = np.frombuffer(image.raw_data, dtype=np.uint8)
                raw = raw.reshape((image.height, image.width, 4)) # BGRA
                bgr = raw[:, :, :3] 
                png_path = output_dir / f"frame_{image.frame:06d}.png"
                cv2.imwrite(str(png_path), bgr)
                saved_first_png = True
                print(f"[OK] first PNG saved: {png_path}")

            print(
                f"[FRAME] frame={image.frame}, ts={image.timestamp:.3f}, "
                f"size={image.width}x{image.height}, raw={len(image.raw_data)}"
            )

        # register the callback function
        camera.listen(on_image)

        wait_sec = 60
        print(f"[INFO] waiting {wait_sec}s for camera frames...")
        time.sleep(wait_sec)

        if len(received_frames) == 0:
            raise RuntimeError("No camera frames received")

        # measure fps
        first_ts = received_frames[0].timestamp
        last_ts = received_frames[-1].timestamp
        duration_sec = last_ts - first_ts

        if duration_sec > 0 and len(received_frames) > 1:
            estimated_fps = (len(received_frames) - 1) / duration_sec
        else:
            estimated_fps = 0.0

        summary_path = output_dir / "summary.txt"
        with open(summary_path, "w", encoding="utf-8") as f:
            f.write(f"received_frames={len(received_frames)}\n")
            f.write(f"first_frame={received_frames[0].frame}\n")
            f.write(f"last_frame={received_frames[-1].frame}\n")
            f.write(f"vehicle_id={vehicle.id}\n")
            f.write(f"camera_id={camera.id}\n")
            f.write(f"vehicle_type={vehicle.type_id}\n")
            f.write(f"camera_type={camera.type_id}\n")
            f.write(f"first_timestamp={first_ts}\n")
            f.write(f"last_timestamp={last_ts}\n")
            f.write(f"estimated_fps={estimated_fps}\n")
            f.write("image_size=1280x720\n")
            f.write("sensor_tick=0.1\n")

        print(f"[OK] received {len(received_frames)} frames")
        print(f"[OK] estimated_fps={estimated_fps:.2f}")
        print(f"[OK] summary saved to {summary_path}")

    except Exception as e:
        print(f"[ERROR] {e}")
        sys.exit(2)

    finally:
        if camera is not None:
            camera.stop()
            camera.destroy()
            print("[CLEANUP] camera destroyed")

        if vehicle is not None:
            vehicle.destroy()
            print("[CLEANUP] vehicle destroyed")


if __name__ == "__main__":
    main()