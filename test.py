import asyncio
from bleak import BleakScanner

async def scan_ble_devices():
    print("正在搜索附近的 BLE 设备，请稍候...")
    
    # 关键修改：使用 discovered_devices_and_advertisement 获取完整信息
    devices_dict = await BleakScanner.discover(return_adv=True)

    if not devices_dict:
        print("未发现任何 BLE 设备。")
        return

    print(f"{'设备名称':<30} | {'设备地址 (MAC/UUID)':<40} | {'信号强度 (RSSI)'}")
    print("-" * 85)

    # devices_dict 的结构是 { "地址": (BLEDevice, AdvertisementData) }
    for address, (device, adv_data) in devices_dict.items():
        name = device.name if device.name else "Unknown Device"
        # 从 adv_data 中获取 rssi
        rssi = adv_data.rssi 
        if (device.address=='84:1F:E8:8D:C9:D6'):
            print(f"{name:<30} | {device.address:<40} | {rssi} dBm")

if __name__ == "__main__":
    try:
        asyncio.run(scan_ble_devices())
    except Exception as e:
        print(f"发生错误: {e}")