# MJPEG-HTTP —— ESP32-S3 标准 MJPEG-over-HTTP 接收固件

由 `v3`(WebSocket 私有协议)改造而来:**不再需要专用推流脚本**,任何能输出标准
MJPEG-over-HTTP 流的软件/App 都可以直接往屏幕上投。

硬件与 `v3` 完全一致(无需改接线):

| 硬件 | 说明 |
|---|---|
| 屏幕 | ST7789 320x240 SPI(MOSI=17 SCLK=18 DC=15 RST=7 CS=6 BL=8) |
| 触摸 | FT6336U I2C(SDA=11 SCL=9 RST=10 INT=13) |
| 解码 | TJpg_Decoder 软件解码,esp_lcd 分块绘制 |

## 目录结构

```
MJPEG-HTTP/
├── mjpeg_http/mjpeg_http.ino   # Arduino 固件(文件夹名与 ino 同名,Arduino 要求)
├── tools/push_screen.py        # 电脑抓屏推流脚本(可选,免 ffmpeg)
└── README.md
```

## 两种工作模式

固件通过 `STREAM_MODE` 宏切换,编译前在 `mjpeg_http.ino` 顶部配置区修改:

### PULL 模式(默认)—— ESP32 去拉别人的流

ESP32 作为 **HTTP 客户端**,持续 GET 拉取 MJPEG 流。适合配合"发送端本身是服务器"的软件:

- **安卓 App「Screen Stream Mirroring」**(免 root):
  1. 手机安装并启动 App,选择"屏幕直播/Start Broadcasting",记下提示的地址(默认端口 8080);
  2. 固件里把 `PULL_HOST` 改成手机 IP(`PULL_PORT=8080` 不用动);
  3. `PULL_PATH` 按版本填 **`/mjpeg` 或 `/stream`**(两者在不同 App 版本中都出现过;
     根路径 `/` 是 App 的网页预览页,不是流——连错时串口会打 text/html 警告);
  4. 编译烧录即可,手机画面实时显示在 ESP32 屏幕上。
- **ffmpeg 作为服务端**(电脑推给 ESP32 拉):
  ```
  ffmpeg -f gdigrab -framerate 15 -i desktop -vf scale=320:240 -q:v 5 \
         -f mpjpeg -listen 1 http://0.0.0.0:8090/stream
  ```
 (macOS 用 `-f avfoundation -i "1:none"`,Linux 用 `-f x11grab -i :0.0`)
  然后固件设 `PULL_HOST=电脑IP, PULL_PORT=8090, PULL_PATH="/stream"`。
- **IP 摄像头**:直接填摄像头的 MJPEG 子码流地址。

### PUSH 模式 —— 别人往 ESP32 推流

把 `STREAM_MODE` 改成 `STREAM_MODE_PUSH`,ESP32 变成 HTTP 服务器(默认端口 8080),
等待客户端 POST/PUT MJPEG 流(ffmpeg 默认的 chunked 传输编码已内置解码):

```
# Windows 电脑屏幕推流
ffmpeg -f gdigrab -framerate 15 -i desktop -vf scale=320:240 -q:v 5 -f mjpeg http://<ESP32_IP>:8080/
```

不用 ffmpeg 也可以用配套脚本:

```
pip install mss pillow
python tools/push_screen.py --esp32 <ESP32_IP> --fps 15 --quality 35
```

用浏览器访问 `http://<ESP32_IP>:8080/` 会看到用法说明页(GET 不推流)。

> `-q:v 5` 越小质量越高;实测 320x240 下 q=5 左右清晰度/码率平衡最好。
> ESP32 待机画面会显示本机 IP 和当前模式,方便核对。

## 编译烧录

- Arduino IDE,开发板 **ESP32S3**,启用 **PSRAM(OPI)**,分区建议 Huge APP;
- 需要安装库:**TJpg_Decoder**(Bodmer),其余(WiFi/Wire)为 esp32 核心自带;
- WiFi 账号在配置区 `ssid` / `password` 修改。

## 协议兼容性说明

流解析**不依赖 boundary 或 Content-Length**,统一扫描 JPEG 的 SOI(`FF D8`)与
EOI(`FF D9`)标记切帧,因此以下封装全部直接兼容:

- raw MJPEG 连续字节流(ffmpeg `-f mjpeg`)
- `multipart/x-mixed-replace`(Screen Stream Mirroring、绝大多数 IP 摄像头)
- `Transfer-Encoding: chunked`(ffmpeg POST 推流默认)

固件还会按**宽高同时**自适应缩放(JPEG 只支持 1/2/4/8 整数缩放),并支持:

- **自动旋转**(`AUTO_ROTATE`,默认开):图像方向与屏幕方向不一致时自动旋转 90°,
  竖屏手机画面投到横屏屏上会转成横向,画面利用率更高;
- **三种显示模式**(`DISPLAY_MODE`):
  - **FIT**:整幅缩放进 320x240 并居中,可能留黑边但内容完整;
  - **CROP(cover)**:尽量放大铺满屏幕后居中裁剪,无黑边但只显示画面中间部分;
  - **RAW(直出)**:缩放/裁剪/旋转全部由发送端完成(直接推 LCD 尺寸如 320x240 的帧),
    ESP32 原样显示、零处理,解码绘制开销和延迟最低。帧尺寸不符时串口会告警;
- 绘制回调支持负偏移裁剪,任意分辨率(横屏/竖屏/超宽)都能正确落到屏幕上。

### 内存与性能

- **乒乓双 JPEG 缓冲 2 x 1MB(PSRAM)**:单帧最大 1MB,基本杜绝大帧丢帧;
  解码与接收各用一块缓冲,互不干扰;
- **行带批量绘制 32 行(内部 SRAM DMA ~20KB)**:解码输出的 MCU 块累积进整行带
  缓冲再一次 SPI 传输,全分辨率帧的 draw 调用从 ~3500 次降到 ~45 次;
- **只解最新帧**:发送端码率超过解码/传输能力时,自动跳过积压的旧帧
  (`[STAT] stale-skip` 计数),延迟稳定在约一帧而不是越积越大;
- **SPI 时钟可调**(`LCD_PCLK_HZ`):默认 40MHz 已验证稳定,追帧率可试 80MHz;
- TCP 读缓冲 8KB(内部 SRAM)。

## 触摸

FT6336U 触摸照常工作,串口会打印坐标。MJPEG 模式没有专用控制通道,
固件支持把触摸坐标以 `GET <TOUCH_PATH>?x=&y=` 上报到指定服务器(配置区
`TOUCH_HOST`,默认禁用)——自建服务端时可以用来做反向触控。

## 串口日志

- `[STREAM]` / `[PUSH]`:连接与响应头信息
- `[DEC]`:每 60 帧的尺寸/耗时/解码结果
- `[STAT]`:每 5 秒的 FPS、丢帧数、内存
- `[TOUCH]`:触摸坐标

## 常见问题

| 现象 | 排查 |
|---|---|
| PULL 一直 connect failed | 检查 `PULL_HOST` IP 是否正确、发送端是否已启动、防火墙 |
| 连上了但没画面 | 看串口响应头,确认 Content-Type 是 image/jpeg 或 multipart/x-mixed-replace |
| 画面不流畅 | 发送端把分辨率缩到 320x240、降低 JPEG 质量/帧率;检查 WiFi 信号(RSSI 日志) |
| `[STAT] dropped` 增长 | 单帧超过 200KB 缓冲上限,降低发送端质量/分辨率 |
| 颜色不对 | 与 v3 相同的 swap/invert 配置已保留,一般不会出现;新屏幕需调 `lcd_init()` |
