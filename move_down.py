import asyncio
from functions import connect_drone, move_down

async def main():
    await connect_drone()
    await move_down(-1)  # Call your async function here

# Get the existing event loop
loop = asyncio.get_event_loop()

# Schedule the main coroutine
loop.create_task(main())

# Note: Do not call loop.run_forever() if the environment already runs the loop

