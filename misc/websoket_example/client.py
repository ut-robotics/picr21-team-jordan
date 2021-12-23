import asyncio
import websockets
import json
import threading
import time


async def listen():
    async with websockets.connect("ws://localhost:8888") as websocket:
        data = await websocket.recv()
        commands = json.loads(data)
        print(commands)

while True:
    loop = asyncio.get_event_loop()
    loop.run_until_complete(listen())
    print("Non blocking")
