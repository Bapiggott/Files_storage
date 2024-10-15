from functions_copy import *
import asyncio
import datetime
import json

original_code = """tu(90);?iv('apple')==True&iv('orange'){l('Yes');->True}l('No');->False;"""

async def main():
    await connect_drone()
    await ensure_armed_and_taken_off()
    await return_to_start_position()
    # Start the print_status_text task
    status_task = asyncio.create_task(print_status_text(drone))

    try:
        await turn_ccw(90)
        if is_visible('apple')==True and is_visible('orange'):
            log('Yes')
            return True
        log('No')
        return False

    finally:
        await stop_offboard_mode()
        status_task.cancel()
        try:
            await status_task
        except asyncio.CancelledError:
            pass

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
