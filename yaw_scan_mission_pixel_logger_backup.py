import asyncio

from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed

from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image

import integrated_stationary_mission as ism


async def wait_connected(drone):
    print("Waiting for drone connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected.")
            return


async def wait_local_position(drone):
    print("Waiting for local position estimate...")
    async for health in drone.telemetry.health():
        print(
            f"global={health.is_global_position_ok}, "
            f"home={health.is_home_position_ok}, "
            f"local={health.is_local_position_ok}"
        )

        if health.is_local_position_ok:
            print("Local position OK.")
            return


async def main():
    print("Starting Gazebo camera subscriber...")
    node = Node()
    node.subscribe(Image, ism.IMAGE_TOPIC, ism.image_callback)

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    await wait_connected(drone)
    await wait_local_position(drone)

    print("Setting takeoff altitude...")
    await drone.action.set_takeoff_altitude(2.5)

    print("Arming...")
    await drone.action.arm()

    print("Taking off...")
    await drone.action.takeoff()

    print("Waiting for takeoff to stabilise...")
    await asyncio.sleep(8)

    print("Setting initial offboard velocity setpoint...")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )

    print("Starting offboard mode...")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard failed: {error._result.result}")
        print("Landing...")
        await drone.action.land()
        return

    print("Starting perception task...")
    perception_task = asyncio.create_task(ism.perception_loop(duration_s=32))

    print("Yaw scan right for 8 seconds...")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 15.0)
    )
    await asyncio.sleep(8)

    print("Yaw scan left for 16 seconds...")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, -15.0)
    )
    await asyncio.sleep(16)

    print("Yaw scan right/center for 8 seconds...")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 15.0)
    )
    await asyncio.sleep(8)

    print("Hovering...")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0)
    )

    summary = await perception_task

    print("Stopping offboard mode...")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard failed: {error._result.result}")

    print("Perception summary:")
    print(summary)

    print("Landing...")
    await drone.action.land()

    print("Yaw scan mission complete.")


if __name__ == "__main__":
    asyncio.run(main())
