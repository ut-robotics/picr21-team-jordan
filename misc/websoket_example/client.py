import asyncio
from websockets import connect
import json

async def game_logic(in_q):
    while True:
        print(in_q)
        await asyncio.sleep(0.0001)


async def listen_websocket(out_q):
    async with connect("ws://localhost:8888") as websocket:
        while True:
            server_data = await websocket.recv()
            command = json.loads(server_data)
            out_q.append(command)


loop = asyncio.get_event_loop()
q = []
loop.create_task(game_logic(q))
loop.create_task(listen_websocket(q))
loop.run_forever()
