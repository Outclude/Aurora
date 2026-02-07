import asyncio
import sys
from bleak import BleakClient

# 配置信息
ADDRESS = "84:1F:E8:8D:C9:D6"
UART_RX_CHAR_UUID = "0000FFF1-0000-1000-8000-00805F9B34FB"
UART_TX_CHAR_UUID = "0000FFF2-0000-1000-8000-00805F9B34FB"

# 收到硬件消息的回调
def notification_handler(sender, data):
    # 仅在收到数据时输出
    msg = data.decode(errors='ignore').strip()
    if msg:
        print(f"\n[硬件 -> 电脑]: {msg}")

async def main(address):
    async with BleakClient(address) as client:
        if not client.is_connected:
            print("❌ 无法连接设备。")
            return

        print(f"✅ 已连接 {address}。输入内容并回车发送，输入 'exit' 退出。")
        
        # 订阅硬件发来的消息
        await client.start_notify(UART_TX_CHAR_UUID, notification_handler)

        loop = asyncio.get_event_loop()
        while True:
            # 异步获取终端输入，不会阻塞蓝牙后台
            message = await loop.run_in_executor(None, sys.stdin.readline)
            message = message.strip()

            if message.lower() == 'exit':
                break
            
            if message:
                try:
                    await client.write_gatt_char(UART_RX_CHAR_UUID, message.encode())
                    # 发送成功后给一个简洁的反馈
                    print(f"[电脑 -> 硬件]: {message}")
                except Exception as e:
                    print(f"发送失败: {e}")

        await client.stop_notify(UART_TX_CHAR_UUID)

if __name__ == "__main__":
    try:
        asyncio.run(main(ADDRESS))
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    except Exception as e:
        print(f"\n程序异常: {e}")