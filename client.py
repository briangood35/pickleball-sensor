import asyncio
from bleak import BleakScanner, BleakClient
import struct

async def main():
    devices = await BleakScanner.discover()
    for d in devices:
        if d.name == "BGood Senso":
            print(d.details)
            print("\n\n")

    async with BleakClient("6ED172EA-63F4-74C2-A8DC-502EE60B7E8E") as client:
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
