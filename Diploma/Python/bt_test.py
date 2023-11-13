from bleak import BleakScanner, BleakClient, BleakError
import asyncio
import funcs
import pickle

CUSTOM_CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
ADDRESS = "A4:C1:38:06:DD:50"

async def main():
    device = await BleakScanner.find_device_by_address(ADDRESS, timeout=30.0)
    if not device:
        raise BleakError(f"A device with address {ADDRESS} could not be found.")
    async with BleakClient(device) as client:
        cMSG = ""
        print("Connected")

        print("Notify start!")
        await client.start_notify(CUSTOM_CHAR_UUID, funcs.notification_handler)
        await asyncio.sleep(10)
        await client.stop_notify(CUSTOM_CHAR_UUID)
        print("Notify end!")
        rMSG = pickle.load(open("rMSG","rb"))
        print(rMSG)

        while(cMSG != "exit"):
            cMSG = input("Message: ")
            await client.write_gatt_char(CUSTOM_CHAR_UUID, cMSG.encode())

asyncio.run(main()) 
