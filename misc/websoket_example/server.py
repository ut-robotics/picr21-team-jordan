import asyncio
import websockets


async def main(websocket, path):
    while True:
        start_blue = '{"signal": "start","targets":  ["001TRT"],"baskets": ["blue"]}'
        input_data = input()
        if input_data == "blue":
            await websocket.send(start_blue)
            
start_server = websockets.serve(main, "localhost", 8888)
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
