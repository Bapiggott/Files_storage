#!/usr/bin/env python3

import asyncio
from mavsdk import System
from mavsdk.action import ActionError


async def run():
    drone = System()
    await drone.connect()#"udp://:14540")

    statustexttask = asyncio.ensure_future(print_status_text(drone))

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    try:
        await drone.action.arm()
    except ActionError as e:
        if e.result == ActionError.Result.ALREADY_ARMED:
            print("-- Already armed")
        else:
            print(f"-- Arming failed with error: {e}")
            return

    print("-- Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(100)

    print("-- Landing")
    await drone.action.land()

    # status_text_task.cancel()


async def print_status_text(drone):
    try:
        async for status_text in drone.telemetry.status_text():
            print(f"Status: {status_text.type}: {status_text.text}")
    except asyncio.CancelledError:
        return


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    for x in range(0, 4):
        # Run the asyncio loop
        loop.run_until_complete(run())

    loop.close()
