#!/usr/bin/env python3

import asyncio
import time

from mavsdk import System
from mavsdk.telemetry import LandedState


async def run():
    drone = System()
    await drone.connect()

    status_text_task = asyncio.ensure_future(print_status_text(drone))

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:

            break

    print("-- Arming")
    await drone.action.arm()

    """async for landed_state in drone.telemetry.landed_state():
        print(f"Landed state: {landed_state}")"""

    print("-- Taking off")
    await drone.action.takeoff()

    print("Starting sleep")
    await asyncio.sleep(10)
    print("Finished sleep")

    print("-- Landing")
    await drone.action.land()

    status_text_task.cancel()

    async for state in drone.telemetry.landed_state():
        if state == LandedState.ON_GROUND:
            break

    print("-- Disarming")
    await drone.action.disarm()

async def print_status_text(drone):
    try:
        async for status_text in drone.telemetry.status_text():
            print(f"Status: {status_text.type}: {status_text.text}")
    except asyncio.CancelledError:
        return


if __name__ == "__main__":
    time_stop = time.monotonic() + 40
    time_start = time_stop - 40

    while time.monotonic() < time_stop:
        asyncio.run(run())
        print(f"Time lapsed: {time.monotonic() - time_start}")
