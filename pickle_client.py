import asyncio
from bleak import BleakScanner, BleakClient
import struct
import PyObjCTools
import os
import sys

def generic_notif_handler(data, type_str):
    if type_str == "accelX":
        os.system('clear')
        sys.stdout.flush()
    type_str += ": "
    print(type_str + str(struct.unpack('<f', data)))
    print()

def handleAccelX(sender, data):
    generic_notif_handler(data, "accelX")

def handleAccelY(sender, data):
    generic_notif_handler(data, "accelY")

def handleAccelZ(sender, data):
    generic_notif_handler(data, "accelZ")

def handleGyroX(sender, data):
    generic_notif_handler(data, "gyroX")

def handleGyroY(sender, data):
    generic_notif_handler(data, "gyroY")

def handleGyroZ(sender, data):
    generic_notif_handler(data, "gyroZ")

def handleTimestamp(sender, data):
    print("timestamp: " + str(struct.unpack('<I', data)[0]))
    print()

async def main():
    devices = await BleakScanner.discover()
    address = None
    for d in devices:
        if d.name == "PickleSens":
            address = str(PyObjCTools.KeyValueCoding.getKey(d.details,'identifier')[0])

    async with BleakClient(address) as client:
        await client.start_notify("12345678-1234-5678-1234-56789abcdef1", handleAccelX)
        await client.start_notify("12345678-1234-5678-1234-56789abcdef2", handleAccelY)
        await client.start_notify("12345678-1234-5678-1234-56789abcdef3", handleAccelZ)
        await client.start_notify("12345678-1234-5678-1234-56789abcdef4", handleGyroX)
        await client.start_notify("12345678-1234-5678-1234-56789abcdef5", handleGyroY)
        await client.start_notify("12345678-1234-5678-1234-56789abcdef6", handleGyroZ)
        await client.start_notify("12345678-1234-5678-1234-56789abcdef7", handleTimestamp)
        while True:
            continue


asyncio.run(main())
