import asyncio
import math
import random
import time

from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan

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

    print_mission_progress_task = asyncio.ensure_future(print_mission_progress(drone))

    running_tasks = [print_mission_progress_task]
    termination_task = asyncio.ensure_future(observe_is_in_air(drone, running_tasks))
    x_coord = []
    y_coord = []
    mission_items = []
    movement = 0.00028076684748779
    starting_x = 37.52280
    starting_y = -122.255604
    speed = 20

    for t in range(0, 10, 1):
        x_coord.append(starting_x + (movement * t * random.uniform(0.5, 1.5)))
        y_coord.append(starting_y + (movement * t * random.uniform(0.5, 1.5)))

    for i in range(10):
        altitude = random.uniform(20, 75)
        speed = random.uniform(1, 5)

        # Provide default values for the missing parameters
        gimbal_pitch_deg = 0
        gimbal_yaw_deg = 0
        camera_action = MissionItem.CameraAction.NONE
        loiter_time_s = 0
        camera_photo_interval_s = 0
        acceptance_radius_m = 1.0
        yaw_deg = 0
        camera_photo_distance_m = 0

        mission_items.append(MissionItem(
            x_coord[i], y_coord[i], altitude, speed, True,
            gimbal_pitch_deg, gimbal_yaw_deg, camera_action,
            loiter_time_s, camera_photo_interval_s, acceptance_radius_m,
            yaw_deg, camera_photo_distance_m
        ))

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
