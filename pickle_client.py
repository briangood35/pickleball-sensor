import asyncio
from bleak import BleakScanner, BleakClient
import struct
import PyObjCTools

async def main():
    devices = await BleakScanner.discover()
    address = None
    for d in devices:
        if d.name == "PickleSens":
            address = str(PyObjCTools.KeyValueCoding.getKey(d.details,'identifier')[0])

    async with BleakClient(address) as client:
        while True:
            accelX = await client.read_gatt_char("12345678-1234-5678-1234-56789abcdef1")
            accelY = await client.read_gatt_char("12345678-1234-5678-1234-56789abcdef2")
            accelZ = await client.read_gatt_char("12345678-1234-5678-1234-56789abcdef3")
            gyroX = await client.read_gatt_char("12345678-1234-5678-1234-56789abcdef4")
            gyroY = await client.read_gatt_char("12345678-1234-5678-1234-56789abcdef5")
            gyroZ = await client.read_gatt_char("12345678-1234-5678-1234-56789abcdef6")
            print(f"accelX: {struct.unpack('<f', accelX)}")
            print(f"accelY: {struct.unpack('<f', accelY)}")
            print(f"accelZ: {struct.unpack('<f', accelZ)}")
            print(f"gyroX: {struct.unpack('<f', gyroX)}")
            print(f"gyroY: {struct.unpack('<f', gyroY)}")
            print(f"gyroZ: {struct.unpack('<f', gyroZ)}")


asyncio.run(main())
