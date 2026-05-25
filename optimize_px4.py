import asyncio
from mavsdk import System

async def run():
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for drone to initialize...")
    async for health in drone.telemetry.health():
        if health.is_accelerometer_calibration_ok:
            print("✅ Drone ready!")
            break

    print("\n--- PX4 TUNING HELPER FOR STABLE x500_depth SITL TESTING ---")
    print("NOTE: This restores standard GPS/Baro-fused flight for reliable simulation.")
    print("Enabling EKF2_GPS_CTRL (Bit 0 = Horizontal)...")
    await drone.param.set_param_int("EKF2_GPS_CTRL", 1)
    
    print("Enabling EKF2_BARO_CTRL (Bit 0 = Enabled)...")
    await drone.param.set_param_int("EKF2_BARO_CTRL", 1)
    
    # 2. Allow Vision for Yaw if available (Bit 2)
    print("Setting EKF2_EV_CTRL to 0 (Default)...")
    await drone.param.set_param_int("EKF2_EV_CTRL", 0)
    
    # 3. Final Safety settings
    print("Allowing arming without GPS (COM_ARM_WO_GPS)...")
    await drone.param.set_param_int("COM_ARM_WO_GPS", 1)
    
    print("\n✅ PX4 RESET TO STABLE. RESTART THE SIMULATION NOW.")

if __name__ == "__main__":
    asyncio.run(run())
