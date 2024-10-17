"""from functions import *
import asyncio
asyncio.run(connect_drone())
#ensure_armed_and_taken_off()
#turn_ccw(90)
move_down(-1)
"""
from functions import *
import asyncio

async def main():
    await connect_drone()
    await move_down(100)
    #await drone.close()
asyncio.run(main())
