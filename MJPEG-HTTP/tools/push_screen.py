#!/usr/bin/env python3
"""
push_screen.py —— 抓取电脑屏幕,以标准 MJPEG-over-HTTP 流推送给 ESP32
(配合 MJPEG-HTTP 固件的 PUSH 模式,等价于 ffmpeg -f mjpeg http://esp32:8080/ 推流)

依赖: pip install mss pillow

用法:
    python push_screen.py --esp32 192.168.x.x            # 默认 8080 端口
    python push_screen.py --esp32 192.168.x.x --fps 15 --quality 35
    python push_screen.py --esp32 192.168.x.x --monitor 2  # 多显示器选第 2 块
"""

import argparse
import http.client
import io
import sys
import time

import mss
from PIL import Image

# mss 10.x 把工厂函数改名了(mss.mss -> mss.MSS),做兼容
_mss_factory = getattr(mss, "MSS", mss.mss)


def main():
    ap = argparse.ArgumentParser(description="Push PC screen as MJPEG-over-HTTP stream to ESP32")
    ap.add_argument("--esp32", required=True, help="ESP32 IP address (PUSH mode)")
    ap.add_argument("--port", type=int, default=8080, help="ESP32 push server port (default 8080)")
    ap.add_argument("--path", default="/", help="HTTP path (default /)")
    ap.add_argument("--fps", type=float, default=12, help="target FPS (default 12)")
    ap.add_argument("--quality", type=int, default=35, help="JPEG quality 1-95 (default 35)")
    ap.add_argument("--monitor", type=int, default=1, help="monitor number, 1=primary (default 1)")
    ap.add_argument("--width", type=int, default=320, help="resize width (default 320)")
    ap.add_argument("--height", type=int, default=240, help="resize height (default 240)")
    args = ap.parse_args()

    conn = http.client.HTTPConnection(args.esp32, args.port, timeout=10)
    conn.putrequest("POST", args.path, skip_accept_encoding=True)
    conn.putheader("Content-Type", "application/octet-stream")
    conn.putheader("Transfer-Encoding", "chunked")   # ESP32 固件支持 chunked 解码
    conn.putheader("Connection", "close")
    conn.endheaders()
    print(f"[INFO] connected, pushing {args.width}x{args.height} @ ~{args.fps}fps "
          f"(quality {args.quality}) to http://{args.esp32}:{args.port}{args.path}")
    print("[INFO] press Ctrl+C to stop")

    frame_count = 0
    stat_start = time.time()
    interval = 1.0 / args.fps

    try:
        with _mss_factory() as sct:
            monitor = sct.monitors[args.monitor]
            while True:
                t0 = time.time()

                sct_img = sct.grab(monitor)
                img = Image.frombytes("RGB", sct_img.size, sct_img.bgra, "raw", "BGRX")
                img = img.resize((args.width, args.height), Image.Resampling.BILINEAR)

                buf = io.BytesIO()
                img.save(buf, format="JPEG", quality=args.quality)
                jpeg = buf.getvalue()

                # chunked 编码发送
                conn.send(f"{len(jpeg):X}\r\n".encode("ascii") + jpeg + b"\r\n")

                frame_count += 1
                if frame_count % 60 == 0:
                    fps = 60 / max(0.001, time.time() - stat_start)
                    print(f"[STAT] sent {frame_count} frames, recent FPS: {fps:.1f}")
                    stat_start = time.time()

                # 按 target FPS 节流(把编码/发送耗时计入)
                elapsed = time.time() - t0
                if elapsed < interval:
                    time.sleep(interval - elapsed)

    except KeyboardInterrupt:
        print("\n[INFO] stopping...")
    except (BrokenPipeError, ConnectionError, OSError) as e:
        print(f"[ERROR] connection lost: {e}")
        sys.exit(1)
    finally:
        try:
            conn.send(b"0\r\n\r\n")          # chunked 结束标记
            resp = conn.getresponse(timeout=3)
            print(f"[INFO] server responded: {resp.status} {resp.reason}")
        except Exception:
            pass
        conn.close()


if __name__ == "__main__":
    main()
