# WS client example
import time
import asyncio
import websockets

async def hello():
    async with websockets.connect('ws://localhost:8765') as websocket:        
        name = 'steve'

        await websocket.send(name)
        print(f"> {name}")

        greeting = await websocket.recv()
        print(f"< {greeting}")
for i in range(5):
    asyncio.get_event_loop().run_until_complete(hello())
    time.sleep(1)