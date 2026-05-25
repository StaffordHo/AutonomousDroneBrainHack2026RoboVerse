from mavsdk import System
from mavsdk.offboard import Offboard
from mavsdk.offboard import VelocityNedYaw, PositionNedYaw
import asyncio
import math
import time

class Drone:
    def __init__(self):
        self.drone = System()

    def _normalize_yaw(self, yaw_deg):
        while yaw_deg > 180:
            yaw_deg -= 360
        while yaw_deg < -180:
            yaw_deg += 360
        return yaw_deg

    def _yaw_error(self, target, current):
        error = target - current
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        return error

    async def connect(self):
        await self.drone.connect(system_address="udpin://0.0.0.0:14540")

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("Connected")
                break

    async def arm_and_takeoff(self):
        await self.drone.action.arm()
        await self.drone.action.takeoff()
        await asyncio.sleep(20)
        print("Takeoff")
        # Required before start
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        # start offboard mode
        await self.drone.offboard.start()

    async def land(self):
        await self.drone.offboard.stop()
        await self.drone.action.land()
        await asyncio.sleep(10)
        print("land")
        await self.drone.action.disarm()

    async def get_position(self):
        async for pos in self.drone.telemetry.position_velocity_ned():
            return pos.position.north_m, pos.position.east_m, pos.position.down_m

    async def get_yaw(self):
        async for att in self.drone.telemetry.attitude_euler():
            return att.yaw_deg

    async def send_velocity(self, vx, vy, vz,yaw_deg):
         await self.drone.offboard.set_velocity_ned(VelocityNedYaw(north_m_s=vx, east_m_s=vy, down_m_s=vz, yaw_deg=yaw_deg))

    async def send_position_setpoint(self, north, east, down, yaw_deg):
        await self.drone.offboard.set_position_ned(PositionNedYaw(north_m=north, east_m=east, down_m=down, yaw_deg=yaw_deg))

    async def rotate_to_yaw(self, target_yaw_deg, tolerance=2.0):
        """
        Rotate to a target yaw using PID control
        """
        target_yaw_deg = self._normalize_yaw(target_yaw_deg)

        # PID gains (start with these, we will tune based on your output)
        Kp = 5.0
        Ki = 0.0
        Kd = 0.2

        integral = 0.0
        prev_error = 0.0
        dt = 0.1  # 10 Hz loop
        max_yaw_rate = 360.0
        
        start = time.monotonic()
        step = 0
        
        # CSV Header for easy copy-paste
        # print("Step,Time(s),Target(deg),Current(deg),Error(deg),P_term,I_term,D_term,Output_Raw(deg/s),Output_Clamped(deg/s),Saturated")

        while True:
            current_yaw = await self.get_yaw()
            error = self._yaw_error(target_yaw_deg, current_yaw)

            # Stop condition
            if abs(error) < tolerance:
                print(f"FINISHED,Step={step},Final_Error={error:.2f},Total_Time={time.monotonic()-start:.2f}")
                break

            # PID terms
            integral += error * dt
            derivative = (error - prev_error) / dt

            p_term = Kp * error
            i_term = Ki * integral
            d_term = Kd * derivative
            output_raw = p_term + i_term + d_term

            # Clamp output
            output = max(min(output_raw, max_yaw_rate), -max_yaw_rate)
            saturated = abs(output_raw) > max_yaw_rate

            # Convert to target yaw step
            new_yaw = current_yaw + output * dt
            new_yaw = self._normalize_yaw(new_yaw)

            elapsed = time.monotonic() - start
            # Print current step data
            # print(f"{step},{elapsed:.2f},{target_yaw_deg:.2f},{current_yaw:.2f},{error:.2f},{p_term:.2f},{i_term:.2f},{d_term:.2f},{output_raw:.2f},{output:.2f},{saturated}")

            # Send command
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(
                    north_m_s=0.0,
                    east_m_s=0.0,
                    down_m_s=0.0,
                    yaw_deg=new_yaw
                )
            )

            prev_error = error
            await asyncio.sleep(dt)
            step += 1

        end = time.monotonic()
        print(f"Turning time: {end - start}")
        # Final stabilization
        await self.drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, target_yaw_deg)
        )

    async def go_to_altitude(self, target_altitude_m, tolerance=0.3):
        """
        Climb/descend to a target altitude (in meters above ground, positive up).
        Uses velocity control with a P controller on altitude error.
        """
        Kp = 1.0
        max_vz = 2.0  # m/s vertical speed cap
        dt = 0.1

        current_yaw = await self.get_yaw()

        while True:
            north, east, down = await self.get_position()
            current_altitude = -down  # NED: down is negative altitude

            error = target_altitude_m - current_altitude

            if abs(error) < tolerance:
                print(f"Reached target altitude: {current_altitude:.2f}m")
                break

            # P controller: positive error = need to go up = negative down velocity
            vz = -Kp * error
            vz = max(min(vz, max_vz), -max_vz)  # clamp

            await self.send_velocity(0.0, 0.0, vz, current_yaw)
            await asyncio.sleep(dt)

        # Hold position at altitude
        await self.send_velocity(0.0, 0.0, 0.0, current_yaw)

    # =========================
    # 🚁 HIGH-LEVEL COMMANDS
    # =========================

    async def turn_cw_90(self):
        current = await self.get_yaw()
        await self.rotate_to_yaw(current + 90)

    async def turn_ccw_90(self):
        current = await self.get_yaw()
        await self.rotate_to_yaw(current - 90)

    async def turn_cw_180(self):
        current = await self.get_yaw()
        await self.rotate_to_yaw(current + 180)