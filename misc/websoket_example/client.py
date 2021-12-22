import asyncio
import websockets
import json


class SocketDataGetter:
    def __init__(self, out_q):
        self.out_q = out_q
        asyncio.get_event_loop().run_until_complete(self.get_data())
        
    async def get_data(self):
        async with websockets.connect("ws://localhost:8888") as websocket:
            while True:
                data = await websocket.recv()
                commands = json.loads(data)
                self.out_q.append(commands)


if __name__ == "__main__":
    q = []
    sock = SocketDataGetter(q)
