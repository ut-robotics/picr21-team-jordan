import asyncio
import websockets

"""
In order to send commands to robot, you can use this code.
"""

async def main(websocket, path):
    print("Connection.")
    while True:
        start_blue = '{"signal": "start","targets":  ["001TRT"],"baskets": ["blue"]}'
        start_rose = '{"signal": "start","targets":  ["001TRT"],"baskets": ["magneta"]}'
        stop = '{"signal": "stop", "targets":  ["001TRT"]}'
        fake = '{"signal": "start","targets":  ["any_id_whatever"],"baskets": ["magneta"]}'

        input_data = str(await asyncio.get_event_loop().run_in_executor(None, input, "input:"))
        if input_data == "blue":
            await websocket.send(start_blue)
        elif input_data == "rose":
            await websocket.send(start_rose)
        elif input_data == "stop":
            await websocket.send(stop)
        elif input_data == "fake":
            await websocket.send(fake)
            
start_server = websockets.serve(main, "localhost", 8888)
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()