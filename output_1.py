from functions_copy import *
import asyncio
import datetime
import json

original_code = """tu(90);?iv('apple')==True&iv('orange'){l('Yes');->True}l('No');->False;"""

async def main():
    await connect_drone()
    await ensure_armed_and_taken_off()
    await turn_ccw(45)
    await goto("person")


if __name__ == '__main__':
    result = asyncio.run(main())
