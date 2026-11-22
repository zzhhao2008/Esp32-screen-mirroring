/***************************************************************************************************
 * ESP32-S3 无线屏幕 —— 标准 MJPEG-over-HTTP 接收固件
 *
 * 由 v3(WebSocket 私有协议)改造而来,显示/触摸硬件部分 1:1 保留:
 *   - 屏幕: ST7789 320x240 SPI(esp_lcd 驱动,引脚与 v3 相同)
 *   - 触摸: FT6336U I2C(I2C 地址 0x38)
 *   - 解码: TJpg_Decoder(软件 JPEG -> RGB565 -> esp_lcd 分块绘制)
 *
 * 协议:标准 MJPEG over HTTP,支持两种工作模式(通过 STREAM_MODE 选择):
 *
 *   [PULL 模式] ESP32 作为 HTTP 客户端,主动 GET 拉取对方的 MJPEG 流。
 *     适合配合"服务端"型发送端:
 *       - 安卓 App "Screen Stream Mirroring"(手机上启动后,ESP32 拉取 http://手机IP:8080/)
 *       - ffmpeg 作为 HTTP 服务端: ffmpeg -f gdigrab ... -f mpjpeg -listen 1 http://0.0.0.0:8090/stream
 *       - IP 摄像头 / 监控摄像头的 MJPEG 子码流地址
 *
 *   [PUSH 模式] ESP32 作为 HTTP 服务器,监听 PUSH_PORT,等待别人 POST/PUT MJPEG 流。
 *     适合配合"客户端"型发送端:
 *       - ffmpeg 推流: ffmpeg -f gdigrab -i desktop -vf scale=320:240 -f mjpeg http://ESP32_IP:8080/
 *         (ffmpeg 默认使用 chunked 传输编码,本固件已内置解码)
 *       - curl:   curl -T screen.mjpeg -H "Content-Type: application/octet-stream" http://ESP32_IP:8080/
 *       - 配套 Python 脚本 tools/push_screen.py(抓取电脑屏幕推流)
 *
 * 流解析不依赖 boundary / Content-Length,统一扫描 JPEG 的 SOI(FF D8)/EOI(FF D9)标记切帧,
 * 因此 raw MJPEG、multipart/x-mixed-replace、chunked 三种封装都能直接吃。
 *
 * 所需 Arduino 库: TJpg_Decoder(Bodmer)  其余(WiFi/Wire)为 esp32 核心自带
 * 板子设置: ESP32S3,开启 PSRAM(OPI),Flash 分区建议 Huge APP
 **************************************************************************************************/

#include <TJpg_Decoder.h>
#include <WiFi.h>
#include <Wire.h>

// ============ 显示层:IDF esp_lcd(与 v3 完全相同) ============
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#define LCD_W 320
#define LCD_H 240
/* 3.2
#define PIN_LCD_MOSI 17
#define PIN_LCD_SCLK 18
#define PIN_LCD_DC   15
#define PIN_LCD_RST  7
#define PIN_LCD_CS   6
#define PIN_LCD_BL   8    // 高电平点亮*/

// 2.4NT
#define PIN_LCD_MOSI 17
#define PIN_LCD_SCLK 18
#define PIN_LCD_DC   15
#define PIN_LCD_RST  16
#define PIN_LCD_CS   14
#define PIN_LCD_BL   13    // 高电平点亮

#define COLOR_BLACK 0x0000
#define COLOR_WHITE 0xFFFF
#define COLOR_RED   0xF800
#define COLOR_GREEN 0x07E0

static esp_lcd_panel_io_handle_t lcd_io = NULL;
static esp_lcd_panel_handle_t lcd_panel = NULL;

// ================= 用户配置区 =================
const char* ssid     = "P70PP";          // WiFi 名称(与 v3 相同)
const char* password = "12345678.";   // WiFi 密码

// 工作模式选择
#define STREAM_MODE_PULL  0
#define STREAM_MODE_PUSH  1
#define STREAM_MODE       STREAM_MODE_PULL   // 改成 STREAM_MODE_PUSH 即切换为推流接收模式

// ---- PULL 模式参数:要去拉流的 MJPEG 地址 ----
const char*    PULL_HOST = "192.168.43.1";  // 发送端 IP(例如开了 Screen Stream Mirroring 的手机)
const uint16_t PULL_PORT = 8080;             // 发送端端口(Screen Stream Mirroring 默认 8080)
const char*    PULL_PATH = "/mjpeg";        // 流路径(Screen Stream Mirroring 为 "/mjpeg";根路径 "/" 是它的网页预览页)

// ---- PUSH 模式参数:本机监听端口,等别人 POST MJPEG 流过来 ----
const uint16_t PUSH_PORT = 8080;

// ---- 显示模式:发送端分辨率与屏幕(320x240)不一致时如何呈现 ----
//  FIT  = 整幅缩放到屏幕内并居中(竖屏画面会上下留黑边,但看得到完整画面)
//  CROP = 尽量放大铺满屏幕后居中裁剪(铺满无黑边,竖屏画面只显示中间部分)
//  RAW  = 直出模式:缩放/裁剪/旋转全部由发送端完成(直接推 LCD 尺寸如 320x240 的帧),
//         ESP32 原样显示,不做任何缩放/旋转/偏移,解码绘制开销最小、延迟最低
#define DISPLAY_MODE_FIT  0
#define DISPLAY_MODE_CROP 1
#define DISPLAY_MODE_RAW  2
#define DISPLAY_MODE      DISPLAY_MODE_RAW

// ---- 自动旋转:图像方向与屏幕方向不一致时,自动旋转 90°(顺时针)再适配
//      竖屏手机画面投到横屏屏上会转成横向,画面利用率更高(FIT 时宽度能占满)
#define AUTO_ROTATE 1   // 1=启用(推荐) 0=关闭

// ---- SPI 时钟:40MHz 已验证稳定;想再提速可改 80*1000*1000(个别屏会花屏,回退即可)
#define LCD_PCLK_HZ (40 * 1000 * 1000)


// ---------- FT6336U I2C 电容触摸配置(与 v3 完全相同) ----------
#define TOUCH_SDA  11
#define TOUCH_SCL  9
#define TOUCH_RST  10
#define TOUCH_INT  12
//#define TOUCH_INT  13  //非触摸的背光驱动与此处冲突
#define FT6336_ADDR  0x38

#define TOUCH_SWAP_XY   true
#define TOUCH_INVERT_X  false
#define TOUCH_INVERT_Y  false

// ---- 触摸上报(可选):非空则把触摸坐标以 GET 发到该地址(路径追加 ?x=&y=)
//      例如 TOUCH_HOST = "192.168.1.100", TOUCH_PORT = 8080, TOUCH_PATH = "/touch"
//      留空 "" 表示禁用(大多数现成 App 没有控制通道,默认禁用)
const char*    TOUCH_HOST = "";
const uint16_t TOUCH_PORT = 8080;
const char*    TOUCH_PATH = "/touch";

// ==============================================

// ---------- 前向声明(供流任务/扫描器提前调用) ----------
void decode_and_draw();
const char* jresult_str(JRESULT r);

// ---------- 内嵌 5x7 ASCII 字体(与 v3 相同) ----------
static const uint8_t FONT5X7[95][5] = {
  0x00, 0x00, 0x00, 0x00, 0x00, // ' '
  0x00, 0x00, 0x5F, 0x00, 0x00, // '!'
  0x00, 0x07, 0x00, 0x07, 0x00, // '"'
  0x14, 0x7F, 0x14, 0x7F, 0x14, // '#'
  0x24, 0x2A, 0x7F, 0x2A, 0x12, // '$'
  0x23, 0x13, 0x08, 0x64, 0x62, // '%'
  0x36, 0x49, 0x56, 0x20, 0x50, // '&'
  0x00, 0x08, 0x07, 0x03, 0x00, // '\''
  0x00, 0x1C, 0x22, 0x41, 0x00, // '('
  0x00, 0x41, 0x22, 0x1C, 0x00, // ')'
  0x2A, 0x1C, 0x7F, 0x1C, 0x2A, // '*'
  0x08, 0x08, 0x3E, 0x08, 0x08, // '+'
  0x00, 0x80, 0x70, 0x30, 0x00, // ','
  0x08, 0x08, 0x08, 0x08, 0x08, // '-'
  0x00, 0x00, 0x60, 0x60, 0x00, // '.'
  0x20, 0x10, 0x08, 0x04, 0x02, // '/'
  0x3E, 0x51, 0x49, 0x45, 0x3E, // '0'
  0x00, 0x42, 0x7F, 0x40, 0x00, // '1'
  0x72, 0x49, 0x49, 0x49, 0x46, // '2'
  0x21, 0x41, 0x49, 0x4D, 0x33, // '3'
  0x18, 0x14, 0x12, 0x7F, 0x10, // '4'
  0x27, 0x45, 0x45, 0x45, 0x39, // '5'
  0x3C, 0x4A, 0x49, 0x49, 0x31, // '6'
  0x41, 0x21, 0x11, 0x09, 0x07, // '7'
  0x36, 0x49, 0x49, 0x49, 0x36, // '8'
  0x46, 0x49, 0x49, 0x29, 0x1E, // '9'
  0x00, 0x00, 0x14, 0x00, 0x00, // ':'
  0x00, 0x40, 0x34, 0x00, 0x00, // ';'
  0x00, 0x08, 0x14, 0x22, 0x41, // '<'
  0x14, 0x14, 0x14, 0x14, 0x14, // '='
  0x00, 0x41, 0x22, 0x14, 0x08, // '>'
  0x02, 0x01, 0x59, 0x09, 0x06, // '?'
  0x3E, 0x41, 0x5D, 0x59, 0x4E, // '@'
  0x7C, 0x12, 0x11, 0x12, 0x7C, // 'A'
  0x7F, 0x49, 0x49, 0x49, 0x36, // 'B'
  0x3E, 0x41, 0x41, 0x41, 0x22, // 'C'
  0x7F, 0x41, 0x41, 0x41, 0x3E, // 'D'
  0x7F, 0x49, 0x49, 0x49, 0x41, // 'E'
  0x7F, 0x09, 0x09, 0x09, 0x01, // 'F'
  0x3E, 0x41, 0x51, 0x73, 0x47, // 'G'
  0x7F, 0x08, 0x08, 0x08, 0x7F, // 'H'
  0x00, 0x41, 0x7F, 0x41, 0x00, // 'I'
  0x20, 0x40, 0x41, 0x3F, 0x01, // 'J'
  0x7F, 0x08, 0x14, 0x22, 0x41, // 'K'
  0x7F, 0x40, 0x40, 0x40, 0x40, // 'L'
  0x7F, 0x02, 0x1C, 0x02, 0x7F, // 'M'
  0x7F, 0x04, 0x08, 0x10, 0x7F, // 'N'
  0x3E, 0x41, 0x41, 0x41, 0x3E, // 'O'
  0x7F, 0x09, 0x09, 0x09, 0x06, // 'P'
  0x3E, 0x41, 0x51, 0x21, 0x5E, // 'Q'
  0x7F, 0x09, 0x09, 0x09, 0x06, // 'R'
  0x26, 0x49, 0x49, 0x49, 0x32, // 'S'
  0x03, 0x01, 0x7F, 0x01, 0x03, // 'T'
  0x3F, 0x40, 0x40, 0x40, 0x3F, // 'U'
  0x1F, 0x20, 0x40, 0x20, 0x1F, // 'V'
  0x3F, 0x40, 0x38, 0x40, 0x3F, // 'W'
  0x63, 0x14, 0x08, 0x14, 0x63, // 'X'
  0x03, 0x04, 0x78, 0x04, 0x03, // 'Y'
  0x61, 0x59, 0x49, 0x4D, 0x43, // 'Z'
  0x00, 0x7F, 0x41, 0x41, 0x41, // '['
  0x02, 0x04, 0x08, 0x10, 0x20, // '\\'
  0x00, 0x41, 0x41, 0x41, 0x7F, // ']'
  0x04, 0x02, 0x01, 0x02, 0x04, // '^'
  0x40, 0x40, 0x40, 0x40, 0x40, // '_'
  0x00, 0x03, 0x07, 0x08, 0x00, // '`'
  0x20, 0x54, 0x54, 0x78, 0x40, // 'a'
  0x7F, 0x28, 0x44, 0x44, 0x38, // 'b'
  0x38, 0x44, 0x44, 0x44, 0x28, // 'c'
  0x38, 0x44, 0x44, 0x44, 0x28, // 'd'
  0x38, 0x54, 0x54, 0x54, 0x18, // 'e'
  0x00, 0x08, 0x7E, 0x09, 0x02, // 'f'
  0x18, 0xA4, 0xA4, 0x9C, 0x78, // 'g'
  0x7F, 0x08, 0x04, 0x04, 0x78, // 'h'
  0x00, 0x44, 0x7D, 0x40, 0x00, // 'i'
  0x20, 0x40, 0x40, 0x3D, 0x00, // 'j'
  0x7F, 0x10, 0x28, 0x44, 0x00, // 'k'
  0x00, 0x41, 0x7F, 0x40, 0x00, // 'l'
  0x7C, 0x04, 0x78, 0x04, 0x78, // 'm'
  0x7C, 0x08, 0x04, 0x04, 0x78, // 'n'
  0x38, 0x44, 0x44, 0x44, 0x38, // 'o'
  0xFC, 0x18, 0x24, 0x24, 0x18, // 'p'
  0x18, 0x24, 0x24, 0x18, 0xFC, // 'q'
  0x7C, 0x08, 0x04, 0x04, 0x08, // 'r'
  0x48, 0x54, 0x54, 0x54, 0x24, // 's'
  0x04, 0x04, 0x3F, 0x44, 0x24, // 't'
  0x3C, 0x40, 0x40, 0x20, 0x7C, // 'u'
  0x1C, 0x20, 0x40, 0x20, 0x1C, // 'v'
  0x3C, 0x40, 0x30, 0x40, 0x3C, // 'w'
  0x44, 0x28, 0x10, 0x28, 0x44, // 'x'
  0x4C, 0x90, 0x90, 0x90, 0x7C, // 'y'
  0x44, 0x64, 0x54, 0x4C, 0x44, // 'z'
  0x00, 0x08, 0x36, 0x41, 0x00, // '{'
  0x00, 0x00, 0x77, 0x00, 0x00, // '|'
  0x00, 0x41, 0x36, 0x08, 0x00, // '}'
  0x02, 0x01, 0x02, 0x04, 0x02, // '~'
};

// ---------- 基础绘图 ----------
static uint16_t *lineBuf = NULL;   // 整行缓冲(DMA 内存)

void fillScreen(uint16_t color) {
  if (!lineBuf) {
    lineBuf = (uint16_t *)heap_caps_malloc(LCD_W * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!lineBuf) { Serial.println("[E] lineBuf alloc failed"); return; }
  }
  for (int i = 0; i < LCD_W; i++) lineBuf[i] = color;
  for (int y = 0; y < LCD_H; y++)
    esp_lcd_panel_draw_bitmap(lcd_panel, 0, y, LCD_W, y + 1, lineBuf);
}

// 在 (x,y) 画一行文字(单行),scale 放大倍数,带背景色
void drawText(int16_t x, int16_t y, const char *s, uint16_t fg, uint16_t bg, uint8_t scale = 2) {
  static uint16_t *cbuf = NULL;
  const int MAXSCALE = 4;
  if (!cbuf) {
    cbuf = (uint16_t *)heap_caps_malloc(6 * 8 * MAXSCALE * MAXSCALE * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!cbuf) { Serial.println("[E] charBuf alloc failed"); return; }
  }
  if (scale > MAXSCALE) scale = MAXSCALE;
  int cw = 6 * scale, ch = 8 * scale;   // 含 1 像素间隔列
  while (*s) {
    uint8_t c = (uint8_t)*s++;
    const uint8_t *glyph = NULL;
    if (c == '\n') { x = 0; y += ch; continue; }
    if (c >= 0x20 && c <= 0x7E) glyph = FONT5X7[c - 0x20];
    for (int row = 0; row < 8; row++) {
      uint8_t bits = (glyph && row < 7) ? glyph[row] : 0;
      for (int col = 0; col < 6; col++) {
        bool on = (col < 5) && (bits & (0x80 >> col));
        uint16_t color = on ? fg : bg;
        for (int sy = 0; sy < scale; sy++) {
          uint16_t *dst = cbuf + (row * scale + sy) * cw + col * scale;
          for (int sx = 0; sx < scale; sx++) dst[sx] = color;
        }
      }
    }
    esp_lcd_panel_draw_bitmap(lcd_panel, x, y, x + cw, y + ch, cbuf);
    x += cw;
  }
}

SemaphoreHandle_t spiMutex;


bool ft6336_read(uint16_t *rawX, uint16_t *rawY) {
  uint8_t buf[5];
  Wire.beginTransmission(FT6336_ADDR);
  Wire.write(0x02);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)FT6336_ADDR, (uint8_t)5) != 5) return false;
  for (uint8_t i = 0; i < 5; i++) buf[i] = Wire.read();

  uint8_t status = buf[0] & 0x0F;
  if (status < 1 || status > 2) return false;

  uint16_t x = ((buf[1] & 0x0F) << 8) | buf[2];
  uint16_t y = ((buf[3] & 0x0F) << 8) | buf[4];
  *rawX = x;
  *rawY = y;
  return true;
}

bool readTouchScreen(uint16_t *sx, uint16_t *sy) {
  if (digitalRead(TOUCH_INT) == HIGH) return false;
  uint16_t rawX, rawY;
  if (!ft6336_read(&rawX, &rawY)) return false;
  if (TOUCH_SWAP_XY) { uint16_t t = rawX; rawX = rawY; rawY = t; }
  if (rawX >= LCD_W)  rawX = LCD_W - 1;
  if (rawY >= LCD_H) rawY = LCD_H - 1;
  *sx = TOUCH_INVERT_X ? (LCD_W - 1 - rawX) : rawX;
  *sy = TOUCH_INVERT_Y ? (LCD_H - 1 - rawY) : rawY;
  return true;
}

// ---------- JPEG 接收缓冲与解码 ----------
// 乒乓双缓冲(PSRAM 充足,各 1MB):解码 drawBuf 的同时,网络数据继续写入 scanBuf,
// 接收不再被解码阻塞,延迟更低;单帧超过 1MB 才会丢帧重新同步
#define JPEG_BUFFER_SIZE 1024 * 1024
static uint8_t* jpegBufA = NULL;     // PSRAM 乒乓缓冲 A
static uint8_t* jpegBufB = NULL;     // PSRAM 乒乓缓冲 B
static uint8_t* scanBuf = NULL;      // 当前接收写入的缓冲
static uint8_t* drawBuf = NULL;      // 最近一帧完整数据所在缓冲
static uint32_t drawLen = 0;         // 最近一帧长度

// ---------- JPEG 流扫描:SOI(FF D8)/EOI(FF D9) 切帧 ----------
// 不依赖 boundary / Content-Length,raw MJPEG / multipart / chunked 通吃
static uint8_t  scanPrev   = 0;      // WAIT_SOI 状态下记录上一个字节
static bool     scanLastFF = false;    // IN_FRAME 状态下上一个字节是否为 0xFF
static uint32_t framePos   = 0;
static bool     inFrame    = false;
static uint32_t droppedFrames = 0;   // 因超过缓冲上限被丢弃的帧数
static uint32_t staleSkipped = 0;    // 因积压(延迟大)被跳过、只解最新帧的帧数

// ---------- 旋转 / 行带批量绘制状态(tft_output 及其辅助函数使用) ----------
static bool    rotEnabled = false;              // 本帧是否旋转 90°(顺时针)
static int32_t rotOffX = 0, rotOffY = 0;        // 旋转后坐标系的目标偏移
static int32_t rotSrcH = 0;                     // 缩放后源图高度(旋转坐标映射用)
void bandReset();
void bandFlush();

// 返回 true 表示收到一帧完整 JPEG(数据已切换到 drawBuf,长度 drawLen;
// scanBuf 已切到另一块乒乓缓冲,可继续接收下一帧)
bool scanner_feed(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    uint8_t b = data[i];
    if (!inFrame) {
      // 等待 SOI: FF D8
      if (scanPrev == 0xFF && b == 0xD8) {
        scanBuf[0] = 0xFF; scanBuf[1] = 0xD8;
        framePos = 2; inFrame = true; scanLastFF = false; scanPrev = 0;
      } else {
        scanPrev = b;
      }
    } else {
      if (framePos < JPEG_BUFFER_SIZE) {
        scanBuf[framePos++] = b;
      } else {
        // 帧太大超过缓冲,丢弃本帧并重新同步
        inFrame = false; scanPrev = 0; scanLastFF = false;
        droppedFrames++;
        continue;
      }
      if (scanLastFF && b == 0xD9) {
        // EOI 到达,完整一帧(FF D9 已写入缓冲)
        // 乒乓切换:该缓冲交给解码,后续数据写入另一块(解码期间接收不停顿)
        inFrame = false; scanPrev = 0;
        drawBuf = scanBuf;
        drawLen = framePos;
        scanBuf = (scanBuf == jpegBufA) ? jpegBufB : jpegBufA;
        return true;
      }
      scanLastFF = (b == 0xFF);
    }
  }
  return false;
}

// ---------- chunked 传输编码解码(ffmpeg 推流默认 chunked) ----------
enum ChunkState { CHUNK_SIZE_LINE, CHUNK_DATA, CHUNK_CRLF, CHUNK_TRAILERS };
static ChunkState chunkState   = CHUNK_SIZE_LINE;
static uint32_t   chunkRemain  = 0;
static char       chunkHex[16];
static uint8_t    chunkHexPos  = 0;
static bool       chunkedEnabled = false;

void chunked_reset() {
  chunkState = CHUNK_SIZE_LINE; chunkRemain = 0; chunkHexPos = 0; chunkedEnabled = false;
}

// 把 chunked 解码后的数据喂给 JPEG 扫描器
// 返回 true 表示本次喂入期间有完整帧到达(是否解码由外层根据积压情况决定)
bool chunked_feed(const uint8_t* data, size_t len) {
  bool gotFrame = false;
  for (size_t i = 0; i < len; i++) {
    uint8_t b = data[i];
    switch (chunkState) {
      case CHUNK_SIZE_LINE:
        if (b == '\n') {
          chunkHex[chunkHexPos] = 0;
          chunkHexPos = 0;
          // 行内可能带 ";ext" 扩展,strtoul 自动在分号处截断
          chunkRemain = strtoul(chunkHex, NULL, 16);
          if (chunkRemain == 0) {
            chunkState = CHUNK_TRAILERS;        // 最后一块,后面全是 trailer,忽略到断开
          } else {
            chunkState = CHUNK_DATA;
          }
        } else if (b != '\r' && chunkHexPos < sizeof(chunkHex) - 1) {
          chunkHex[chunkHexPos++] = (char)b;
        }
        break;

      case CHUNK_DATA: {
        size_t n = len - i;
        if (n > chunkRemain) n = chunkRemain;
        if (scanner_feed(data + i, n)) gotFrame = true;
        i += n - 1;
        chunkRemain -= n;
        if (chunkRemain == 0) chunkState = CHUNK_CRLF;
        break;
      }

      case CHUNK_CRLF:
        if (b == '\n') chunkState = CHUNK_SIZE_LINE;   // 吃掉 \r\n
        break;

      case CHUNK_TRAILERS:
        // 忽略直到连接关闭
        break;
    }
  }
  return gotFrame;
}

// 统一入口:原始字节流 -> (按需去 chunked) -> JPEG 扫描
// 返回 true 表示有完整帧到达(drawBuf/drawLen 即该帧;是否解码由外层决定)
// 注意:解码必须在本函数返回 true 的"当下"进行——继续喂入会覆盖缓冲
bool body_feed(const uint8_t* data, size_t len) {
  if (!chunkedEnabled) {
    return scanner_feed(data, len);
  }
  return chunked_feed(data, len);
}

// ---------- 触摸上报(可选) ----------
uint32_t lastTouchReport = 0;

void reportTouch(uint16_t x, uint16_t y) {
  if (TOUCH_HOST[0] == 0) return;                     // 未配置则不上报
  if (millis() - lastTouchReport < 100) return;       // 限频 10Hz,避免刷爆对端
  lastTouchReport = millis();

  WiFiClient c;
  c.setTimeout(2000);
  if (!c.connect(TOUCH_HOST, TOUCH_PORT)) return;
  c.printf("GET %s?x=%u&y=%u HTTP/1.1\r\n", TOUCH_PATH, x, y);
  c.printf("Host: %s:%u\r\n", TOUCH_HOST, TOUCH_PORT);
  c.print("Connection: close\r\n\r\n");
  // 不等响应,直接断开(固件端单向通知)
  c.stop();
}

// ---------- 流任务(PULL 或 PUSH) ----------
#if STREAM_MODE == STREAM_MODE_PULL

// ---- PULL:HTTP 客户端,持续拉取 MJPEG 流 ----
void streamTask(void *pvParameters) {
  static uint8_t rbuf[8192];
  while (1) {
    if (WiFi.status() != WL_CONNECTED) { vTaskDelay(500 / portTICK_PERIOD_MS); continue; }

    WiFiClient client;
    client.setNoDelay(true);
    Serial.printf("[STREAM] connecting http://%s:%u%s ...\n", PULL_HOST, PULL_PORT, PULL_PATH);
    if (!client.connect(PULL_HOST, PULL_PORT)) {
      Serial.println("[STREAM] connect failed, retry in 2s");
      vTaskDelay(2000 / portTICK_PERIOD_MS);
      continue;
    }

    // 发送 GET 请求
    client.printf("GET %s HTTP/1.1\r\n", PULL_PATH);
    client.printf("Host: %s:%u\r\n", PULL_HOST, PULL_PORT);
    client.print("User-Agent: ESP32-MJPEG-Receiver/1.0\r\n");
    client.print("Accept: multipart/x-mixed-replace, image/jpeg, */*\r\n");
    client.print("Connection: close\r\n\r\n");

    // 读响应头(最多 8KB / 8 秒)
    String headers;
    uint32_t hStart = millis();
    bool headerOK = false;
    while (client.connected() && millis() - hStart < 8000) {
      if (!client.available()) { vTaskDelay(1); continue; }
      headers += (char)client.read();
      if (headers.endsWith("\r\n\r\n")) { headerOK = true; break; }
      if (headers.length() > 8192) break;
    }
    if (!headerOK) {
      Serial.println("[STREAM] no/bad response headers, retry");
      client.stop();
      vTaskDelay(2000 / portTICK_PERIOD_MS);
      continue;
    }
    Serial.println("[STREAM] ---- response headers ----");
    Serial.print(headers);
    Serial.println("[STREAM] ----------------------------");

    // 是否 chunked(大小写不敏感检查)
    String hl; hl.reserve(headers.length());
    for (unsigned i = 0; i < headers.length(); i++) hl += (char)tolower(headers[i]);
    chunked_reset();
    chunkedEnabled = (hl.indexOf("transfer-encoding") >= 0 && hl.indexOf("chunked") >= 0);
    if (chunkedEnabled) Serial.println("[STREAM] Transfer-Encoding: chunked -> de-chunk enabled");

    // 友好提示:连到的是 HTML 网页而不是 MJPEG 流
    if (hl.indexOf("content-type: text/html") >= 0) {
      Serial.println("[STREAM] WARNING: got text/html (a web page, not the stream).");
      Serial.println("[STREAM] Try PULL_PATH = \"/stream\" (Screen Stream Mirroring) or the direct stream URL.");
    }

    // 持续读 body,扫描 JPEG 帧并解码
    while (client.connected()) {
      int avail = client.available();
      if (avail <= 0) { vTaskDelay(1); continue; }
      int n = client.read(rbuf, (avail > (int)sizeof(rbuf)) ? (int)sizeof(rbuf) : avail);
      if (n <= 0) continue;
      if (body_feed(rbuf, (size_t)n)) {
        // 只解最新帧:积压超过一帧就跳过,尽快排空旧数据,把延迟压到约一帧
        if (client.available() > (int)framePos) {
          staleSkipped++;
        } else {
          decode_and_draw();
        }
      }
    }

    Serial.println("[STREAM] server closed connection, reconnecting...");
    client.stop();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

#else

// ---- PUSH:HTTP 服务器,接收 POST/PUT 的 MJPEG 流 ----
WiFiServer pushServer(PUSH_PORT);

void pushSendSimple(WiFiClient &c, int code, const char* text) {
  char hdr[160];
  snprintf(hdr, sizeof(hdr),
           "HTTP/1.1 %d %s\r\nContent-Type: text/plain\r\nContent-Length: %d\r\nConnection: close\r\n\r\n%s",
           code, text, (int)strlen(text), text);
  c.print(hdr);
}

void streamTask(void *pvParameters) {
  static uint8_t rbuf[8192];
  pushServer.begin();
  Serial.printf("[PUSH] HTTP server listening on port %u (POST/PUT MJPEG stream here)\n", PUSH_PORT);

  while (1) {
    if (WiFi.status() != WL_CONNECTED) { vTaskDelay(500 / portTICK_PERIOD_MS); continue; }

    WiFiClient client = pushServer.available();
    if (!client) { vTaskDelay(1); continue; }
    client.setNoDelay(true);
    Serial.printf("[PUSH] client %s:%d connected\n",
                  client.remoteIP().toString().c_str(), client.remotePort());

    // 读请求头
    String headers;
    uint32_t hStart = millis();
    bool headerOK = false;
    while (millis() - hStart < 10000) {
      if (!client.connected()) break;
      if (!client.available()) { vTaskDelay(1); continue; }
      headers += (char)client.read();
      if (headers.endsWith("\r\n\r\n")) { headerOK = true; break; }
      if (headers.length() > 8192) break;
    }
    if (!headerOK) {
      Serial.println("[PUSH] request header timeout/oversize, dropping client");
      client.stop();
      continue;
    }
    Serial.println(headers);

    // 小写副本做大小写不敏感检查
    String hl; hl.reserve(headers.length());
    for (unsigned i = 0; i < headers.length(); i++) hl += (char)tolower(headers[i]);

    bool isPost = hl.startsWith("post") || hl.startsWith("put");
    bool expect100 = (hl.indexOf("expect:") >= 0 && hl.indexOf("100-continue") >= 0);

    if (!isPost) {
      // GET:给浏览器看一个简单说明页
      pushSendSimple(client, 200,
        "ESP32 MJPEG push receiver.\nSend MJPEG stream with POST, e.g.:\n"
        "ffmpeg -f gdigrab -i desktop -vf scale=320:240 -f mjpeg http://<esp32-ip>:8080/");
      client.flush();
      client.stop();
      continue;
    }

    // ffmpeg 等 POST 客户端可能带 Expect: 100-continue,必须先回 100 才会发数据
    if (expect100) {
      client.print("HTTP/1.1 100 Continue\r\n\r\n");
      Serial.println("[PUSH] sent 100 Continue");
    }

    chunked_reset();
    chunkedEnabled = (hl.indexOf("transfer-encoding") >= 0 && hl.indexOf("chunked") >= 0);
    if (chunkedEnabled) Serial.println("[PUSH] Transfer-Encoding: chunked -> de-chunk enabled");

    // 流式读 body,扫描 JPEG 帧并解码
    while (client.connected()) {
      int avail = client.available();
      if (avail <= 0) { vTaskDelay(1); continue; }
      int n = client.read(rbuf, (avail > (int)sizeof(rbuf)) ? (int)sizeof(rbuf) : avail);
      if (n <= 0) continue;
      if (body_feed(rbuf, (size_t)n)) {
        // 只解最新帧:积压超过一帧就跳过,尽快排空旧数据,把延迟压到约一帧
        if (client.available() > (int)framePos) {
          staleSkipped++;
        } else {
          decode_and_draw();
        }
      }
    }

    // 客户端推完断开,补一个最终响应(可能发不出去,尽力而为)
    client.print("HTTP/1.1 200 OK\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
    client.flush();
    client.stop();
    Serial.println("[PUSH] client done");
  }
}

#endif

// ---------- 解码 + 绘制(由流任务在收到完整帧后调用) ----------
static uint32_t statFrames = 0;
static uint32_t statWindowFrames = 0;
static uint32_t statWindowStart = 0;

void decode_and_draw() {
  if (drawBuf == NULL || drawLen < 4) return;

  uint32_t t0 = micros();

  uint16_t w = 0, h = 0;
  TJpgDec.getJpgSize(&w, &h, drawBuf, drawLen);
  if (w == 0 || h == 0) {
    Serial.println("[DEC] getJpgSize failed, skip frame");
    return;
  }

  bool rotate = false;
  uint8_t scale = 1;
  int32_t oxR = 0, oyR = 0;

#if DISPLAY_MODE == DISPLAY_MODE_RAW
  // RAW 直出:发送端已完成缩放/裁剪/旋转(应为 LCD 尺寸帧),原样显示,零处理
  if ((w != LCD_W || h != LCD_H) && statFrames % 60 == 0) {
    Serial.printf("[DEC] WARNING: RAW mode expects %dx%d frames, got %ux%u (resize on sender)\n",
                  LCD_W, LCD_H, w, h);
  }
#else
  // 自动旋转:图像方向与屏幕方向不一致时旋转 90°(顺时针)
  //   竖屏手机画面(高>宽)投到横屏屏(宽>高)上 -> 转成横向,宽边能占满屏幕
#if AUTO_ROTATE
  rotate = ((w > h) != (LCD_W > LCD_H));
#endif

  // 旋转后的等效图像尺寸(缩放前)
  uint32_t iw = rotate ? h : w;
  uint32_t ih = rotate ? w : h;

  // 按"宽高同时"自适应缩放(JPEG 解码只支持 1/2/4/8 整数缩放)
#if DISPLAY_MODE == DISPLAY_MODE_FIT
  // 整幅缩放到屏幕内:不断加倍 scale 直到宽高都装得下(最多 1/8)
  while (scale < 8 && (iw > (uint32_t)LCD_W * scale || ih > (uint32_t)LCD_H * scale)) {
    scale *= 2;
  }
#else
  // CROP:选最小的 scale 让缩放后的宽高都 >= 屏幕(铺满),居中后超出部分被回调裁掉
  if (iw <= LCD_W && ih <= LCD_H) {
    scale = 1;   // 图比屏幕还小,原尺寸居中即可
  } else {
    for (uint8_t s = 1; s <= 8; s *= 2) {
      scale = s;
      if (iw / s >= LCD_W && ih / s >= LCD_H) break;
    }
  }
#endif

  // 居中偏移(按旋转后的坐标系计算)
  oxR = ((int32_t)LCD_W - (int32_t)(iw / scale)) / 2;
  oyR = ((int32_t)LCD_H - (int32_t)(ih / scale)) / 2;
#endif

  TJpgDec.setJpgScale(scale);
  rotEnabled = rotate;
  rotSrcH    = (int32_t)(h / scale);   // 缩放后源图高(旋转映射用)
  rotOffX    = oxR;
  rotOffY    = oyR;

  xSemaphoreTake(spiMutex, portMAX_DELAY);
  bandReset();
  JRESULT r;
  if (rotate) {
    // 旋转模式:drawJpg 不加偏移,目标偏移在回调里做坐标映射
    r = TJpgDec.drawJpg(0, 0, drawBuf, drawLen);
  } else {
    r = TJpgDec.drawJpg(oxR, oyR, drawBuf, drawLen);
  }
  bandFlush();   // 冲刷行带批量绘制缓冲(仍持有 spiMutex)
  xSemaphoreGive(spiMutex);

  statFrames++;
  statWindowFrames++;
  uint32_t dt = micros() - t0;

  // 每 60 帧打一条详细日志
  if (statFrames % 60 == 1) {
    Serial.printf("[DEC] #%lu %ux%u%s scale1/%u off(%d,%d) %lu bytes result=%s decode+draw=%lu us stack HW:%u\n",
                  (unsigned long)statFrames, w, h, rotate ? " ROT90" : "", scale,
                  (int)oxR, (int)oyR,
                  (unsigned long)drawLen, jresult_str(r), (unsigned long)dt,
                  (unsigned)uxTaskGetStackHighWaterMark(NULL));
  }

  // 每 5 秒打一条 FPS 统计
  uint32_t now = millis();
  if (statWindowStart == 0) statWindowStart = now;
  if (now - statWindowStart >= 5000) {
    float fps = statWindowFrames * 1000.0f / (now - statWindowStart);
    Serial.printf("[STAT] %.1f fps | total=%lu stale-skip=%lu dropped=%lu | heap=%u KB psram=%u KB\n",
                  fps, (unsigned long)statFrames, (unsigned long)staleSkipped,
                  (unsigned long)droppedFrames,
                  ESP.getFreeHeap() / 1024, ESP.getFreePsram() / 1024);
    statWindowStart = now;
    statWindowFrames = 0;
  }
}

// JRESULT -> 可读文本(tjpgd 错误码)
const char* jresult_str(JRESULT r) {
  switch (r) {
    case JDR_OK:   return "OK";
    case JDR_INTR: return "INTR (interrupted by output function)";
    case JDR_INP:  return "INP (insufficient input)";
    case JDR_MEM1: return "MEM1 (workspace insufficient for image)";
    case JDR_MEM2: return "MEM2 (streaming buffer insufficient)";
    case JDR_PAR:  return "PAR (parameter error)";
    case JDR_FMT1: return "FMT1 (broken JPEG data)";
    case JDR_FMT2: return "FMT2 (right format but not supported)";
    case JDR_FMT3: return "FMT3 (not supported JPEG standard)";
    default:       return "UNKNOWN";
  }
}

// ---------- JPEG 解码输出回调 ----------
// 三条路径:
//   1. 非旋转 + 行带批量绘制:MCU 块累积进整行带缓冲,一行带一次 SPI 传输
//      (全分辨率帧从 ~3500 次 draw 降到 ~90 次,是绘制环节最大的提速点)
//   2. 旋转 90°(AUTO_ROTATE):块级像素转置后绘制(转置块 ≤16x16)
//   3. 兜底:任意情况下裁剪后直接绘制
// 注意:本回调及行带函数都在 decode_and_draw 持有 spiMutex 的上下文中执行,
//       内部不要再取互斥量(非递归锁会死锁)。
static uint16_t *blockBuf = NULL;   // 块搬运缓冲(DMA)
static uint16_t *rowBand  = NULL;   // 行带缓冲:LCD_W x 32 像素(内部 SRAM DMA,~20KB)
static uint16_t *rotBuf   = NULL;   // 转置缓冲(DMA)
#define BLOCK_BUF_PX  1024
#define BAND_ROWS     32

static int32_t bandY = -1;          // 当前行带起始行(-1 = 无)
static int32_t bandW = 0;           // 当前行带已填充宽度
static int32_t bandBottom = 0;      // 当前行带已填充底部行

void bandReset() { bandY = -1; bandW = 0; bandBottom = 0; }

// 冲刷当前行带:整块一次 draw_bitmap
void bandFlush() {
  if (bandY < 0 || bandW == 0 || bandBottom <= bandY) { bandReset(); return; }
  esp_lcd_panel_draw_bitmap(lcd_panel, 0, bandY, bandW, bandBottom, rowBand);
  bandReset();
}

// 裁剪到屏幕范围(支持负偏移/越界,步长 stride 与宽度 w 解耦)后绘制
static bool drawBlockClipped(int32_t x, int32_t y, uint16_t w, uint16_t h,
                             const uint16_t* pixels, uint16_t stride) {
  if (x < 0) { int32_t cut = -x; if (cut >= (int32_t)w) return true; pixels += cut; w -= cut; x = 0; }
  if (y < 0) { int32_t cut = -y; if (cut >= (int32_t)h) return true; pixels += (int32_t)cut * stride; h -= cut; y = 0; }
  if (x >= LCD_W || y >= LCD_H) return true;
  if (x + w > LCD_W) w = LCD_W - x;
  if (y + h > LCD_H) h = LCD_H - y;
  if (w == 0 || h == 0) return true;

  if (!blockBuf) {
    blockBuf = (uint16_t *)heap_caps_malloc(BLOCK_BUF_PX * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!blockBuf) { Serial.println("[E] blockBuf alloc failed"); return false; }
  }
  if ((uint32_t)w * h <= BLOCK_BUF_PX) {
    // 小块:先整块拷进 DMA 缓冲(按 stride 取行),一次 SPI 传输
    for (uint16_t row = 0; row < h; row++)
      memcpy(blockBuf + (uint32_t)row * w, pixels + (uint32_t)row * stride, (uint32_t)w * 2);
    esp_lcd_panel_draw_bitmap(lcd_panel, x, y, x + w, y + h, blockBuf);
  } else {
    // 超大块兜底:逐行传输
    for (uint16_t row = 0; row < h; row++) {
      memcpy(blockBuf, pixels + (uint32_t)row * stride, (uint32_t)w * 2);
      esp_lcd_panel_draw_bitmap(lcd_panel, x, y + row, x + w, y + row + 1, blockBuf);
    }
  }
  return true;
}

// 非旋转路径:把裁剪后的块累积进行带(解码器按光栅顺序输出,同一 MCU 行天然连续)
static bool bandAccum(int16_t x, int16_t y, uint16_t w, uint16_t h,
                      const uint16_t* pixels, uint16_t stride) {
  if (!rowBand) {
    rowBand = (uint16_t *)heap_caps_malloc(LCD_W * BAND_ROWS * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!rowBand) return false;
  }
  // 能接进当前行带:同一 16 行带内,且 x 与游标连续
  if (!(bandY >= 0 && y >= bandY && y + h <= bandY + BAND_ROWS && x == bandW)) {
    bandFlush();
    bandY = ((int32_t)y / BAND_ROWS) * BAND_ROWS;
    bandW = 0;
    bandBottom = bandY;
    if (y < bandY || y + h > bandY + BAND_ROWS) return false;   // 异常对齐 → 直接绘制兜底
  }
  uint16_t *dst = rowBand + (uint32_t)(y - bandY) * LCD_W + x;
  for (uint16_t row = 0; row < h; row++)
    memcpy(dst + (uint32_t)row * LCD_W, pixels + (uint32_t)row * stride, (uint32_t)w * 2);
  if (x + w > bandW) bandW = x + w;
  if (y + h > bandBottom) bandBottom = y + h;
  return true;
}

// 旋转 90°(顺时针)路径:源块 (x,y,w,h) -> 目标块宽高互换 + 像素转置
// 映射:dest(i列,j行) = src(第 h-1-i 行, 第 j 列);dest_x = rotOffX + rotSrcH - (y+h)
static bool tft_output_rot(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  int32_t dx = rotOffX + rotSrcH - (int32_t)y - (int32_t)h;
  int32_t dy = rotOffY + (int32_t)x;
  uint16_t dw = h, dh = w;   // 目标块:dw 列 x dh 行

  if (!rotBuf) {
    rotBuf = (uint16_t *)heap_caps_malloc(BLOCK_BUF_PX * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!rotBuf) { Serial.println("[E] rotBuf alloc failed"); return false; }
  }
  for (uint16_t j = 0; j < dh; j++)
    for (uint16_t i = 0; i < dw; i++)
      rotBuf[(uint32_t)j * dw + i] = bitmap[(uint32_t)(h - 1 - i) * w + j];
  return drawBlockClipped(dx, dy, dw, dh, rotBuf, dw);
}

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  if (rotEnabled) return tft_output_rot(x, y, w, h, bitmap);

  // 非旋转:先裁剪到屏幕范围(负偏移支持,步长 stride 与 w 解耦),再尽量累积行带
  const uint16_t* pixels = bitmap;
  uint16_t stride = w;
  if (x < 0) { int32_t cut = -x; if (cut >= (int16_t)w) return 1; pixels += cut; w -= cut; x = 0; }
  if (y < 0) { int32_t cut = -y; if (cut >= (int16_t)h) return 1; pixels += (int32_t)cut * stride; h -= cut; y = 0; }
  if (x >= LCD_W || y >= LCD_H) return 1;
  if (x + w > LCD_W) w = LCD_W - x;
  if (y + h > LCD_H) h = LCD_H - y;
  if (w == 0 || h == 0) return 1;

  if (!bandAccum(x, y, w, h, pixels, stride)) {
    if (!drawBlockClipped(x, y, w, h, pixels, stride)) return 0;
  }
  return 1;
}

// ---------- LCD 初始化(与 v3 完全相同) ----------
bool lcd_init() {
  pinMode(PIN_LCD_BL, OUTPUT);
  digitalWrite(PIN_LCD_BL, HIGH);

  spi_bus_config_t buscfg = {
    .mosi_io_num = PIN_LCD_MOSI,
    .miso_io_num = -1,
    .sclk_io_num = PIN_LCD_SCLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = LCD_W * 2,
    .flags = SPICOMMON_BUSFLAG_MASTER,
  };
  esp_err_t ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
  Serial.printf("[LCD] spi_bus_initialize: %s\n", esp_err_to_name(ret));
  if (ret != ESP_OK) return false;

  esp_lcd_panel_io_spi_config_t io_config = {
    .cs_gpio_num = PIN_LCD_CS,
    .dc_gpio_num = PIN_LCD_DC,
    .spi_mode = 3,
    .pclk_hz = LCD_PCLK_HZ,
    .trans_queue_depth = 10,
    .on_color_trans_done = NULL,
    .user_ctx = NULL,
    .lcd_cmd_bits = 8,
    .lcd_param_bits = 8,
  };
  ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &io_config, &lcd_io);
  Serial.printf("[LCD] new_panel_io_spi: %s\n", esp_err_to_name(ret));
  if (ret != ESP_OK) return false;

  esp_lcd_panel_dev_config_t panel_config = {
    .reset_gpio_num = PIN_LCD_RST,
    .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
    .bits_per_pixel = 16,
  };
  ret = esp_lcd_new_panel_st7789(lcd_io, &panel_config, &lcd_panel);
  Serial.printf("[LCD] new_panel_st7789: %s\n", esp_err_to_name(ret));
  if (ret != ESP_OK) return false;

  ret = esp_lcd_panel_reset(lcd_panel);
  Serial.printf("[LCD] panel_reset: %s\n", esp_err_to_name(ret));
  if (ret != ESP_OK) return false;
  ret = esp_lcd_panel_init(lcd_panel);
  Serial.printf("[LCD] panel_init: %s\n", esp_err_to_name(ret));
  if (ret != ESP_OK) return false;

  esp_lcd_panel_invert_color(lcd_panel, true);
  esp_lcd_panel_swap_xy(lcd_panel, true);
  esp_lcd_panel_mirror(lcd_panel, false, true);
  esp_lcd_panel_disp_on_off(lcd_panel, true);
  delay(100);
  Serial.println("[LCD] ST7789 initialized (LuatOS QLCD style)");
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n[BOOT] MJPEG-HTTP starting (esp_lcd display)...");
  Serial.printf("[BOOT] SDK: %s\n", ESP.getSdkVersion());

  spiMutex = xSemaphoreCreateMutex();
  if (spiMutex == NULL) {
    Serial.println("Error: Failed to create SPI mutex!");
    while (1) delay(100);
  }

  jpegBufA = (uint8_t*)ps_malloc(JPEG_BUFFER_SIZE);
  jpegBufB = (uint8_t*)ps_malloc(JPEG_BUFFER_SIZE);
  if (jpegBufA == NULL || jpegBufB == NULL) {
    Serial.println("Error: Failed to allocate JPEG ping-pong buffers in PSRAM!");
    Serial.printf("[BOOT] PSRAM size: %u KB\n", ESP.getPsramSize() / 1024);
    while (1) delay(100);
  }
  scanBuf = jpegBufA;
  drawBuf = NULL;
  Serial.printf("[BOOT] PSRAM OK: %u KB total, ping-pong jpeg buffers 2 x %u KB\n",
                ESP.getPsramSize() / 1024, JPEG_BUFFER_SIZE / 1024);

  if (!lcd_init()) {
    Serial.println("Error: LCD init failed!");
    while (1) delay(100);
  }

  fillScreen(COLOR_BLACK);
  drawText(10, 10, "Booting...", COLOR_WHITE, COLOR_BLACK, 2);

  // 初始化 FT6336U 触摸
  pinMode(TOUCH_RST, OUTPUT);
  digitalWrite(TOUCH_RST, LOW);
  delay(10);
  digitalWrite(TOUCH_RST, HIGH);
  delay(50);
  pinMode(TOUCH_INT, INPUT);
  Wire.begin(TOUCH_SDA, TOUCH_SCL);
  Wire.setClock(400000);
  Wire.beginTransmission(FT6336_ADDR);
  if (Wire.endTransmission() == 0) {
    Serial.println("[TOUCH] FT6336U found on I2C bus.");
  } else {
    Serial.println("[TOUCH] Warning: FT6336U not found! Check SDA=11/SCL=9 wiring.");
  }

  TJpgDec.setJpgScale(1);
  TJpgDec.setSwapBytes(true);
  TJpgDec.setCallback(tft_output);

  // 连接 WiFi(30 秒超时)
  drawText(10, 40, "Waiting WiFi...", COLOR_WHITE, COLOR_BLACK, 2);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);
  Serial.printf("[WIFI] Connecting to %s", ssid);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - t0 > 30000) {
      Serial.println("\nError: WiFi connect timeout (30s)! Check SSID/password.");
      drawText(10, 70, "WiFi timeout!", COLOR_RED, COLOR_BLACK, 2);
      while (1) delay(100);
    }
  }
  Serial.printf("\n[WIFI] Connected! IP: %s, RSSI: %d dBm\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI());

  // 待机画面:显示模式与地址
  fillScreen(COLOR_BLACK);
  drawText(10, 10, "MJPEG-HTTP ready", COLOR_GREEN, COLOR_BLACK, 2);
  drawText(10, 40, WiFi.localIP().toString().c_str(), COLOR_WHITE, COLOR_BLACK, 2);
#if STREAM_MODE == STREAM_MODE_PULL
  {
    char line[48];
    snprintf(line, sizeof(line), "PULL %s:%u", PULL_HOST, PULL_PORT);
    drawText(10, 70, line, COLOR_WHITE, COLOR_BLACK, 2);
  }
#else
  {
    char line[48];
    snprintf(line, sizeof(line), "PUSH port %u", PUSH_PORT);
    drawText(10, 70, line, COLOR_WHITE, COLOR_BLACK, 2);
  }
#endif

  xTaskCreatePinnedToCore(streamTask, "StreamTask", 16384, NULL, 2, NULL, 1);
  Serial.println("[BOOT] Setup done, entering loop.");
}

void loop() {
  // 触摸读取:仅上报(可选 HTTP GET),MJPEG 模式没有专用控制通道
  uint16_t tx, ty;
  if (readTouchScreen(&tx, &ty)) {
    Serial.printf("[TOUCH] %u, %u\n", tx, ty);
    reportTouch(tx, ty);
  }

  // WiFi 断线检测
  static bool wifiWasUp = true;
  bool wifiUp = (WiFi.status() == WL_CONNECTED);
  if (wifiWasUp && !wifiUp) {
    Serial.println("[WIFI] Disconnected! Auto-reconnecting...");
    xSemaphoreTake(spiMutex, portMAX_DELAY);
    fillScreen(COLOR_BLACK);
    drawText(10, 10, "WiFi lost...", COLOR_RED, COLOR_BLACK, 2);
    xSemaphoreGive(spiMutex);
  }
  if (!wifiWasUp && wifiUp) {
    Serial.printf("[WIFI] Reconnected! IP: %s\n", WiFi.localIP().toString().c_str());
    fillScreen(COLOR_BLACK);
    drawText(10, 10, WiFi.localIP().toString().c_str(), COLOR_WHITE, COLOR_BLACK, 2);
  }
  wifiWasUp = wifiUp;

  delay(10);
}
