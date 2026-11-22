import asyncio
import websockets
import mss
from PIL import Image
import io
from pynput.mouse import Button, Controller

ESP32_IP = "192.168.1.43" # Change your Esp32 IP Adress
ESP32_PORT = 81
MONITOR_NUMBER = 1

TARGET_FPS = 12
JPEG_QUALITY = 35
CHUNK_SIZE = 4096          # 单个二进制分块大小（ESP32 WebSockets 库对小块更稳）
FRAME_DONE_TIMEOUT = 2.0   # 等 ESP32 画完一帧的超时（秒），超时则继续发下一帧
SEND_TIMEOUT = 5.0         # 单次网络发送超时（秒），防止 TCP 卡死时无限阻塞
RESIZE_METHOD = Image.Resampling.BILINEAR # You can select BILINEAR/NEAREST

mouse = Controller()

# mss 10.x 把工厂函数改名了（mss.mss -> mss.MSS），做兼容
_mss_factory = getattr(mss, "MSS", mss.mss)

# ESP32 画完一帧后广播 "FRAME_DONE"，用 Event 做流控
frame_done = asyncio.Event()

async def handle_touch(websocket):
    """接收 ESP32 的消息：TOUCH 转成鼠标移动，FRAME_DONE 设置流控事件"""
    try:
        async for message in websocket:
            if isinstance(message, bytes):
                continue
            if message.startswith("TOUCH:"):
                try:
                    coords = message.strip().split(':')[1]
                    x_touch, y_touch = map(int, coords.split(','))
                    with _mss_factory() as sct:
                        monitor = sct.monitors[MONITOR_NUMBER]
                        pc_screen_width = monitor["width"]
                        pc_screen_height = monitor["height"]
                    target_x = (x_touch / 320) * pc_screen_width
                    target_y = (y_touch / 240) * pc_screen_height
                    mouse.position = (int(target_x), int(target_y))
                    # mouse.click(Button.left, 1) # If you want touch touch = click enable this line
                except Exception:
                    pass
            elif message == "FRAME_DONE":
                frame_done.set()
    except websockets.exceptions.ConnectionClosed:
        pass

async def send_one_frame(websocket, sct, monitor_to_capture):
    """抓屏 -> 编码 JPEG -> 发送 -> 等待 ESP32 画完（流控）"""
    start_time = asyncio.get_event_loop().time()

    sct_img = sct.grab(monitor_to_capture)
    resized_img = Image.frombytes("RGB", sct_img.size, sct_img.bgra, "raw", "BGRX").resize((320, 240), RESIZE_METHOD)

    buffer = io.BytesIO()
    resized_img.save(buffer, format="JPEG", quality=JPEG_QUALITY)
    jpeg_data = buffer.getvalue()

    # 发送都带超时：TCP 被卡死时抛异常触发重连，而不是无限阻塞
    await asyncio.wait_for(
        websocket.send(f"JPEG_FRAME_SIZE:{len(jpeg_data)}"), SEND_TIMEOUT)
    for i in range(0, len(jpeg_data), CHUNK_SIZE):
        await asyncio.wait_for(
            websocket.send(jpeg_data[i:i + CHUNK_SIZE]), SEND_TIMEOUT)

    # 流控：等 ESP32 画完这一帧（超时兜底继续），保证 ESP32 侧缓冲不积压
    frame_done.clear()
    try:
        await asyncio.wait_for(frame_done.wait(), FRAME_DONE_TIMEOUT)
    except asyncio.TimeoutError:
        pass

    # TARGET_FPS 作为上限节流（抓屏/编码耗时已计入）
    elapsed_time = asyncio.get_event_loop().time() - start_time
    sleep_time = max(0, (1 / TARGET_FPS) - elapsed_time)
    if sleep_time > 0:
        await asyncio.sleep(sleep_time)

async def send_screen(websocket):
    print("Connection is Successful! JPEG sending...")
    try:
        with _mss_factory() as sct:
            monitor_to_capture = sct.monitors[MONITOR_NUMBER]
            print(f"Capture started: Display {MONITOR_NUMBER}")

            frame_count = 0
            stat_start = asyncio.get_event_loop().time()
            while True:
                await send_one_frame(websocket, sct, monitor_to_capture)
                frame_count += 1
                if frame_count % 60 == 0:
                    now = asyncio.get_event_loop().time()
                    fps = 60 / max(0.001, now - stat_start)
                    print(f"[STAT] sent {frame_count} frames this session, recent FPS: {fps:.1f}")
                    stat_start = now
                    frame_count = 0
    except (websockets.exceptions.ConnectionClosed, asyncio.TimeoutError):
        print("Screen share connection lost (timeout or closed).")
        raise
    except OSError as e:
        print(f"Network error: {e}")
        raise

async def main():
    uri = f"ws://{ESP32_IP}:{ESP32_PORT}"
    while True:
        try:
            print(f"Connecting to ESP32: {uri}")
            # compression=None: ESP32 的 WebSocketsServer 不支持 permessage-deflate
            async with websockets.connect(
                uri,
                ping_interval=None,
                max_size=None,
                compression=None,
                open_timeout=10,
                close_timeout=2,
            ) as websocket:
                frame_done.clear()
                send_task = asyncio.create_task(send_screen(websocket))
                touch_task = asyncio.create_task(handle_touch(websocket))
                await asyncio.gather(send_task, touch_task)
        except (websockets.exceptions.ConnectionClosed, ConnectionRefusedError):
            print("Connection failed or lost. Retrying in 2 seconds...")
            await asyncio.sleep(2)
        except asyncio.TimeoutError:
            print("Connection timed out. Retrying in 2 seconds...")
            await asyncio.sleep(2)
        except Exception as e:
            print(f"Unexpected Error: {e}")
            await asyncio.sleep(2)

if __name__ == "__main__":
    asyncio.run(main())
