import asyncio
from mavsdk import System


async def wait_connected(drone):
    print("Waiting for drone connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected.")
            return


async def main():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    await wait_connected(drone)

    print("Waiting for health checks...")
    async for health in drone.telemetry.health():
        print(
            f"global={health.is_global_position_ok}, "
            f"home={health.is_home_position_ok}, "
            f"local={health.is_local_position_ok}"
        )

        # For SITL, local position is usually enough for offboard-style work.
        if health.is_local_position_ok:
            print("Local position OK.")
            break

    print("Setting takeoff altitude...")
    await drone.action.set_takeoff_altitude(2.5)

    print("Arming...")
    await drone.action.arm()

    print("Taking off...")
    await drone.action.takeoff()

    print("Hovering for 8 seconds...")
    await asyncio.sleep(8)

    print("Landing...")
    await drone.action.land()

    print("Done.")


if __name__ == "__main__":
    asyncio.run(main())
