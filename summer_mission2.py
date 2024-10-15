import asyncio
from mavsdk import System

async def run():
    # Create a drone system object and connect
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # Wait for the drone to have a global position estimate
    print("-- Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    # Arm the drone
    print("-- Arming")
    await drone.action.arm()

    # Take off
    print("-- Taking off")
    await drone.action.takeoff()

    # Wait for the drone to reach a stable altitude
    await asyncio.sleep(10)

    # Check that the mission is uploaded and acknowledged by the drone
    print("-- Uploading mission")
    upload_progress = await drone.mission.upload_mission()

    if upload_progress:
        print("-- Mission uploaded successfully")
    else:
        print("Mission upload failed")
        return

    # Start the mission
    print("-- Starting mission")
    try:
        await drone.mission.start_mission()
        print("Mission started successfully")
    except Exception as e:
        print(f"Failed to start mission: {e}")
        return

    # Monitor mission progress
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: {mission_progress.current}/{mission_progress.total}")
        if mission_progress.current == mission_progress.total:
            print("-- Mission complete")
            break

    # Return to launch (RTL) after mission completion
    print("-- Returning to launch")
    await drone.action.return_to_launch()

    # Disarm the drone after landing
    print("-- Disarming")
    await drone.action.disarm()

if __name__ == "__main__":
    asyncio.run(run())


