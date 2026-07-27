#!/usr/bin/env python

import asyncio
import websockets
import os
import ntcore
import time
import vgamepad as vg

FORWARD_PORT = '5000'
TEAM = 3824
PAY_TO_PLAY_PATH = "C:\\Users\\3824\\Documents\\Offseason2026\\tooling\\PayToPlay\\"
PRESS_DURATION_SEC = 0.2

# # Initialize comms with robot
# inst = ntcore.NetworkTableInstance.getDefault()
# table = inst.getTable("PayToPlay")
# numTagsPublisher = table.getDoubleTopic("numTags").publish()
# inst.startClient4("PayToPlayClient")
# # inst.setServerTeam(TEAM)
# inst.setServer("127.0.0.1")
# # inst.startDSClient()

gamepad = vg.VX360Gamepad()

async def press():
    gamepad.press_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_A)
    await asyncio.sleep(PRESS_DURATION_SEC)
    gamepad.release_button(button=vg.XUSB_BUTTON.XUSB_GAMEPAD_A)

async def handler(websocket):
    print("Client connected")

    # ping
    async def ping():
        while True:
            await asyncio.sleep(10)
            await websocket.ping()

    ping_task = asyncio.create_task(ping())

    # send msg based on an integer received from the client
    # async def send_msg(count):
    #     for i in range(1, count + 1):
    #         msg = "hello " + str(i)
    #         print('Sended: ' + msg)
    #         await websocket.send(msg)
    #         await asyncio.sleep(3)

    # send_task = None

    try:
        # listen msg
        while True:
            gamepad.update()
            raw_msg = await websocket.recv()
            print('Received: ' + raw_msg)
            
            # expect the request to be an integer
            try:
                newCount = int(raw_msg)
                if (newCount != count):
                    asyncio.create_task(press())

                count = newCount
                # if numTagsPublisher is not None:
                #     numTagsPublisher.set(count)
                # else:
                #     print("[WARNING] PUBLISHER IS NULL!")
            except ValueError:
                await websocket.send("error: expected an integer")
                continue

            # start (or restart) sending hello messages based on the integer
            # if send_task is not None:
            #     send_task.cancel()
            # send_task = asyncio.create_task(send_msg(count))

    finally:
        ping_task.cancel()
        # if send_task is not None:
        #     send_task.cancel()
        print("end")

async def main():
    # Make sure we're the only ones using the server
    os.system(PAY_TO_PLAY_PATH + "\\platform-tools\\adb kill-server")
    os.system(PAY_TO_PLAY_PATH + "\\platform-tools\\adb start-server")
    # only reverse is needed now — the device connects back to our server
    os.system(PAY_TO_PLAY_PATH + "\\platform-tools\\adb reverse tcp:" + FORWARD_PORT + " tcp:" + FORWARD_PORT)

    async with websockets.serve(handler, "localhost", int(FORWARD_PORT)):
        print(f"Server listening on ws://localhost:{FORWARD_PORT}")
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    asyncio.run(main())