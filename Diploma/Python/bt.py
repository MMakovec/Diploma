from bleak import BleakScanner, BleakClient, BleakError
import asyncio
import funcs
import pathfind

location = False

CUSTOM_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
ADDRESS = "A4:C1:38:06:DD:50"

async def connect():

    global location
    target = False
    finished = False
    # computer sending instructions (only computer -> location from frame)
    computer = False
    # robot sending location (only robot -> guide runs on robot)
    robot = True

    device = await BleakScanner.find_device_by_address(ADDRESS, timeout=3)
    if not device:
        raise BleakError(f"A device with address {ADDRESS} could not be found.")
    async with BleakClient(device) as client:
        print("Connected")

        # Start listening, when data arrives call notification_handler
        await client.start_notify(CUSTOM_CHAR_UUID, notification_handler1)
        print("Notify start!")

        if computer and not robot:
            # Initialise frame
            multitouch_client = funcs.optitouch_multi()

        while True:
            # time for reading robot responses
            await asyncio.sleep(0.01)
            if computer:
                if not target:
                    target = input("Select target coordinates (x,y): ")
                    target = target.split(',')
                    if robot:
                        await client.write_gatt_char(CUSTOM_CHAR_UUID, "s".encode())
                        await asyncio.sleep(0.1)
                    else:
                        location = funcs.frame(multitouch_client)

                if location:
                    print(f"Recieved location: {location}")
                    spd = funcs.guide(location, target)

                    if spd == [0,0]:
                        print("Target reached!")
                        target = False
                        location = False

                    # Repeat if not recieved
                    while not spd:
                        location = []
                        await client.write_gatt_char(CUSTOM_CHAR_UUID, "s".encode())
                        await asyncio.sleep(0.1)
                        print(location)
                        spd = funcs.guide(location, target)

                    msg = f"{spd[0]:.2f},{spd[1]:.2f}"
                    print(f"msg: {msg}")
                    await asyncio.sleep(0.01)
                    await client.write_gatt_char(CUSTOM_CHAR_UUID, msg.encode())
                    if robot:
                        location = False
                    else:
                        location = funcs.frame(multitouch_client)
                
            else:
                if not target:
                    target = True
                    path = False
                    await client.write_gatt_char(CUSTOM_CHAR_UUID, "s".encode())
                    await asyncio.sleep(0.1)
                    block_width = 0.1

                    path = pathfind.main(pathfind.WIDTH, pathfind.HEIGHT)
                    path = path * block_width

                if location:
                    if len(path) == 0:
                        target = False
                        print("Finished")
                        input("press enter to continue")
                        return
                        continue
                    print(f'Location: {location}')
                    print(f'Path: {path}')
                    location = False
                    point, path = path[-1], path[:-1]
                    msg = f"{point[0]:.2f},{-point[1]:.2f}"
                    print(f"Current point: {msg}")
                    await client.write_gatt_char(CUSTOM_CHAR_UUID, msg.encode())
                '''
                uMSG = await funcs.ainput("User interrupt command: ")
                if uMSG == "exit":
                    await client.write_gatt_char(CUSTOM_CHAR_UUID, "R".encode())
                    await client.disconnect()
                    break
                elif uMSG == "stop":
                    await client.write_gatt_char(CUSTOM_CHAR_UUID, "s".encode())
                    input("Press enter to continue: ")
                elif uMSG:
                    await client.write_gatt_char(CUSTOM_CHAR_UUID, uMSG.encode())
                
                '''

def notification_handler(sender, data):
    global location
    print(f'{sender}: {data}')
    if not location:
        location = data.decode().split(',')

def notification_handler1(sender, data):
    global location
    print(f'{sender}: {data}')
    if data.decode() == '\x01':
        location = data.decode().split(',')

if __name__ == "__main__":
    asyncio.run(connect()) 
