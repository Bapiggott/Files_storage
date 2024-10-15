#from functions import *
#from functions_copy import *
from functions_gps import *
import asyncio
import datetime
import json

original_code = """?s('bottle')==True{g('bottle');_2=oh('bottle');l(_2);tp};"""

async def main():
    await connect_drone()
    await ensure_armed_and_taken_off()
    #await move_in_direction(1, "backward")
    #await move_forward(0.2)
    #await check_connection()
    take_picture()
    await goto("bench")
    #await turn_cw(25)
    #await check_connection()
    await move_backward(0.2)
    #await  turn_ccw(25)
    #await move_backward(1)
    #await stop_offboard_mode()
    """await return_to_start_position()
    # Start the print_status_text task
    status_task = asyncio.create_task(print_status_text(drone))
    
    try:
        if await scan('bottle')==True:
            await goto('bottle')
            _2 = object_height('bottle')
            log(_2)
            take_picture()

    finally:
        status_task.cancel()
        try:
            await status_task
        except asyncio.CancelledError:
            pass"""

if __name__ == '__main__':
    result = asyncio.run(main())
        current_datetime = datetime.datetime.now().isoformat()
        with open(__file__, 'r') as f_in:
            translated_code = f_in.read()
    log_data = {
        'date': current_datetime,
        'original_code': original_code,
        'translated_code': translated_code,
        'output': result
    }
    with open('execution_log.json', 'a') as log_file:
        log_file.write(json.dumps(log_data) + '\n')
