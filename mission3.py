import asyncio
import math

from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)
import random
from mavsdk.telemetry import LandedState
import time


async def run():
    drone = System()
    print("connecting ?")
    await drone.connect(system_address="udp://:14540")
    param2 = drone.param
    print("connected")
    await param2.set_param_float('MPC_LAND_SPEED', round(random.uniform(0.6, 1.2), 1))
    mpc_land_speed = await param2.get_param_float('MPC_LAND_SPEED')
    print(f"MPC_LAND_SPEED value: {mpc_land_speed}")

    await param2.set_param_float('MIS_TAKEOFF_ALT', round(random.uniform(20, 75), 0))
    mis_takeoff_alt = await param2.get_param_float('MIS_TAKEOFF_ALT')
    print(f"MIS_TAKEOFF_ALT value: {mis_takeoff_alt}")

    await param2.set_param_float('MPC_TKO_SPEED', round(random.uniform(1, 5), 1))
    mpc_tko_speed = await param2.get_param_float('MPC_TKO_SPEED')
    print(f"MPC_TKO_SPEED value: {mpc_tko_speed}")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print_mission_progress_task = asyncio.ensure_future(
        print_mission_progress(drone))

    running_tasks = [print_mission_progress_task]
    termination_task = asyncio.ensure_future(
        observe_is_in_air(drone, running_tasks))
    
    radius = 0.0002  # adjust the radius of the helix
    vertical_speed = 1  # adjust the vertical speed of the helix

    mission_items = []
    num_points = 20  # Number of points in the helix
    for t in range(num_points):
        angle = 2 * math.pi * t / num_points
        x = 37.52280 + radius * math.cos(angle)
        y = -122.255604 + radius * math.sin(angle)
        z = vertical_speed * t
        mission_items.append(MissionItem(x, y, z, 20, True))

    mission_plan = MissionPlan(mission_items)

    await drone.mission.set_return_to_launch_after_mission(True)

    print("-- Uploading mission")
    await drone.mission.upload_mission(mission_plan)

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Starting mission")
    await drone.mission.start_mission()

    await termination_task

    print("-- Disarming")
    await drone.action.disarm()


async def print_mission_progress(drone):
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: {mission_progress.current}/{mission_progress.total}")


async def observe_is_in_air(drone, running_tasks):
    """ Monitors whether the drone is flying or not and
    returns after landing """

    was_in_air = False

    async for is_in_air in drone.telemetry.in_air():
        if is_in_air:
            was_in_air = is_in_air

        if was_in_air and not is_in_air:
            for task in running_tasks:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
            await asyncio.get_event_loop().shutdown_asyncgens()

            return


if __name__ == "__main__":
    time_stop = time.monotonic() + 210
    time_start = time_stop - 210
    count = 0

    while time.monotonic() < time_stop:
        count += 1
        asyncio.run(run())
        print(f"Time lapsed: {time.monotonic() - time_start}")
    print(f"Total mission ran: {count}")







