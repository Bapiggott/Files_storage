import asyncio
import math
import subprocess
import random
from mavsdk import System
from mavsdk.mission import (MissionItem, MissionPlan)
from mavsdk.telemetry import LandedState
import time
from datetime import datetime

def start_wireshark_capture():
    # Generate a unique filename for each capture using a timestamp
    timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
    pcap_filename = f"mission_capture_{timestamp}.pcap"
    pdml_filename = f"mission_capture_{timestamp}.pdml"

    # Start a new Wireshark capture with the generated filename and specified filter
    try:
        # subprocess.Popen(['wireshark', '-i', 'any', '-k', '-f', 'not icmp and udp port 14550', '-w', pcap_filename])
        subprocess.Popen(['wireshark', '-i', 'any', '-k', '-f', 'mavlink', '-w', pcap_filename])
    except FileNotFoundError:
        print(
            "Wireshark not found in PATH. Please install Wireshark or provide the full path to the Wireshark executable.")

    return pcap_filename


async def print_mission_progress(drone):
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: "
              f"{mission_progress.current}/"
              f"{mission_progress.total}")


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


async def run():
    timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
    pcap_filename = f"mission_capture_{timestamp}.pcap"
    pdml_filename = f"mission_capture_{timestamp}.pdml"
    start_wireshark_capture(pcap_filename)

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
    x_coord = []
    y_coord = []
    mission_items = []
    movement = 0.00028076684748779
    starting_x = 37.52280  # 47.398039859999997
    starting_y = -122.255604  # 8.5455725400000002
    speed = 20
    b = 0
    for t in range(0, 3, 1):
        # Calculate the x and y coordinates using parametric equations
        x_coord.append(starting_x + (((1.5 * t * math.cos(math.pi * t)) / 15) * movement * random.uniform(0.5, 1.5)))
        y_coord.append(starting_y + (movement * t) * random.uniform(0.5, 1.5))
        print(x_coord[b])
        print(y_coord[b])
        print("-------------------")
        b += 1

    for i in range(0, 3, 1):
        mission_items.append(MissionItem(x_coord[i],
                                         y_coord[i],
                                         random.uniform(1, 1.5) * .15 * i * math.cos(math.pi * i) + 30,
                                         speed,
                                         True,
                                         float('nan'),
                                         float('nan'),
                                         MissionItem.CameraAction.NONE,
                                         float('nan'),
                                         float('nan'),
                                         float('nan'),
                                         (math.cos(i * math.pi) * random.randint(10, 40)),
                                         float('nan')))

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

    # Save the Wireshark capture at the end of the mission
    subprocess.run(['killall', 'dumpcap'])  # Stop Wireshark capture
    subprocess.run(['wireshark', '-r', pcap_filename, '-T', 'pdml', '-w', pdml_filename])

if __name__ == "__main__":
    time_stop = time.monotonic() + 210
    time_start = time_stop - 210
    count = 0
    print("hello")

    loop = asyncio.get_event_loop()  # Get the event loop
    while time.monotonic() < time_stop:
        count += 1
        timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
        pcap_filename = f"mission_capture_{timestamp}.pcap"

        # Create an asyncio task within the event loop
        loop.create_task(start_wireshark_capture(pcap_filename))
        loop.run_until_complete(run())  # Run the main coroutine within the event loop
        print(f"Time lapsed: {time.monotonic() - time_start}")
    print(f"Total mission ran: {count}")