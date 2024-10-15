import asyncio
from mavsdk import System

roll = ""
pitch = ""
throttle = ""
yaw = ""

async def main():
    """Main function to connect to the drone and input manual controls"""
    global roll, pitch, yaw, throttle
    # Connect to the Simulation
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # This waits till a mavlink based drone is connected
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break
