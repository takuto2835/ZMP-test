# threading
import threading
import time
import asyncio

async def scheduler(interval, f, loopCheckFunction, wait=True):
    base_time = time.perf_counter()
    next_time = 0

    while loopCheckFunction():
        t = threading.Thread(target=f)
        t.start()
        if wait:
            t.join()
        next_time = ((base_time - time.perf_counter()) % interval) or interval
        await asyncio.sleep(next_time)
