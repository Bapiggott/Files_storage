import asyncio
from mavsdk import System
from mavsdk import (MissionItem, MissionPlan)

async def run():
    # Connect to the UAV
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # Check if the drone is ready
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break

    # Create a list of waypoints for the rectangular path
    mission_items = [
        MissionItem(47.3977421, 8.5455945, 35, 5, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE, float('nan'), float('nan')),
        MissionItem(47.3977421, 8.5455945, 35, 5, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE, float('nan'), float('nan')),
        MissionItem(47.3977421, 8.5457945, 35, 5, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE, float('nan'), float('nan')),
        MissionItem(47.3977421, 8.5457945, 35, 5, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE, float('nan'), float('nan')),
        MissionItem(47.3979421, 8.5457945, 35, 5, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE, float('nan'), float('nan')),
        MissionItem(47.3979421, 8.5455945, 35, 5, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE, float('nan'), float('nan')),
        MissionItem(47.3977421, 8.5455945, 35, 5, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE, float('nan'), float('nan'))
    ]

    # Upload the mission to the drone
    mission_plan = MissionPlan(mission_items)
    await drone.mission.set_return_to_launch_after_mission(True)
    await drone.mission.upload_mission(mission_plan)

    # Start the mission
    await drone.mission.start_mission()

    # Monitor mission progress
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: {mission_progress.current_item_index}/{mission_progress.mission_count}")

    # Wait for the mission to complete
    await asyncio.sleep(10)

    # Stop the mission
    await drone.mission.cancel_mission()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())

