#include <TJpg_Decoder.h>
#include <WiFi.h>
// 开启 WebSockets 库调试输出（排障用，定位后可删掉这两行关闭）
#ifndef DEBUG_ESP_PORT
#define DEBUG_ESP_PORT Serial
#endif
#include <WebSocketsServer.h>
#include <Wire.h>

// ============ 显示层：IDF esp_lcd（替代 TFT_eSPI，Arduino 3.1.1/IDF5.3 下 TFT_eSPI 寄存器级代码不工作） ============
// 初始化流程 1:1 复刻已验证可行的 LuatOS QLCD 方案（qlcd.c / devices.lua）
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#define LCD_W 320
#define LCD_H 240

#define PIN_LCD_MOSI 17
#define PIN_LCD_SCLK 18
#define PIN_LCD_DC   15
#define PIN_LCD_RST  7
#define PIN_LCD_CS   6
#define PIN_LCD_BL   8    // 高电平点亮

// 颜色（标准 RGB565，与 TFT_eSPI 常量一致）
#define COLOR_BLACK 0x0000
#define COLOR_WHITE 0xFFFF
#define COLOR_RED   0xF800
#define COLOR_GREEN 0x07E0
#define COLOR_BLUE  0x001F

static esp_lcd_panel_io_handle_t lcd_io = NULL;
static esp_lcd_panel_handle_t lcd_panel = NULL;

// ---------- 内嵌 5x7 ASCII 字体（标准 Adafruit GLCD 字体，0x20~0x7E） ----------
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
  0x3E, 0x41, 0x41, 0x51, 0x73, // 'G'
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
  0x7F, 0x09, 0x19, 0x29, 0x46, // 'R'
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
  0x38, 0x44, 0x44, 0x28, 0x7F, // 'd'
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
static uint16_t *lineBuf = NULL;   // 整行缓冲（DMA 内存）

void fillScreen(uint16_t color) {
  if (!lineBuf) {
    lineBuf = (uint16_t *)heap_caps_malloc(LCD_W * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!lineBuf) { Serial.println("[E] lineBuf alloc failed"); return; }
  }
  for (int i = 0; i < LCD_W; i++) lineBuf[i] = color;
  for (int y = 0; y < LCD_H; y++)
    esp_lcd_panel_draw_bitmap(lcd_panel, 0, y, LCD_W, y + 1, lineBuf);
}

// 在 (x,y) 画一行文字（单行），scale 放大倍数，带背景色
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

// ---------- WiFi / WebSocket ----------
const char* ssid = "1701";
const char* password = "13964398089";
const int websocket_port = 81;

WebSocketsServer webSocket = WebSocketsServer(websocket_port);

SemaphoreHandle_t spiMutex;

// ---------- FT6336U I2C 电容触摸配置 ----------
// 真实 GPIO：SCL=9, SDA=11, RST=10, INT=13（ES8311 音频芯片与触摸共用同一条 I2C 总线）
#define TOUCH_SDA  11
#define TOUCH_SCL  9
#define TOUCH_RST  10   // 触摸复位引脚
#define TOUCH_INT  13   // 触摸中断引脚（低电平表示有触点）
#define FT6336_ADDR  0x38   // FT6336U 的 I2C 地址，个别板子是 0x39

// 触摸方向映射：如果触摸方向不对，翻转下面两个值即可
// 如果 X/Y 轴完全对不上（横向移动屏幕竖向变化），把 TOUCH_SWAP_XY 改为 true
#define TOUCH_SWAP_XY   true
#define TOUCH_INVERT_X  true
#define TOUCH_INVERT_Y  false

// 读取 FT6336U 第一个触点（原始坐标：X 0~239，Y 0~319）
bool ft6336_read(uint16_t *rawX, uint16_t *rawY) {
  uint8_t buf[5];
  Wire.beginTransmission(FT6336_ADDR);
  Wire.write(0x02);                       // TD_STATUS 寄存器
  if (Wire.endTransmission(false) != 0) return false;   // repeated start
  if (Wire.requestFrom((uint8_t)FT6336_ADDR, (uint8_t)5) != 5) return false;
  for (uint8_t i = 0; i < 5; i++) buf[i] = Wire.read();

  uint8_t status = buf[0] & 0x0F;
  if (status < 1 || status > 2) return false;           // 无有效触点

  uint16_t x = ((buf[1] & 0x0F) << 8) | buf[2];         // TOUCH1_XH/XL
  uint16_t y = ((buf[3] & 0x0F) << 8) | buf[4];         // TOUCH1_YH/YL
  *rawX = x;
  *rawY = y;
  return true;
}

// 把触摸原始坐标映射为当前屏幕方向（横屏 320x240）坐标
bool readTouchScreen(uint16_t *sx, uint16_t *sy) {
  // INT 引脚：低电平表示有触点，高电平直接跳过 I2C 读取
  if (digitalRead(TOUCH_INT) == HIGH) return false;

  uint16_t rawX, rawY;
  if (!ft6336_read(&rawX, &rawY)) return false;

  if (TOUCH_SWAP_XY) {
    uint16_t t = rawX; rawX = rawY; rawY = t;
  }
  // 此时 rawX 对应屏幕横轴（0~319），rawY 对应屏幕纵轴（0~239）
  if (rawX >= LCD_W)  rawX = LCD_W - 1;
  if (rawY >= LCD_H) rawY = LCD_H - 1;

  *sx = TOUCH_INVERT_X ? (LCD_W - 1 - rawX) : rawX;
  *sy = TOUCH_INVERT_Y ? (LCD_H - 1 - rawY) : rawY;
  return true;
}

// ---------- JPEG 解码输出：直接用 esp_lcd 绘制 ----------
#define JPEG_BUFFER_SIZE 60 * 1024
uint8_t* jpeg_buffer1 = NULL;
uint8_t* jpeg_buffer2 = NULL;

uint32_t jpeg_buffer_pos = 0;
uint32_t expected_jpeg_size = 0;

uint32_t frame_count = 0;

// 帧接收状态：事件回调只负责收数据，解码/绘制由 websocketTask 主循环执行
// （避免在 webSocket.loop() 回调内部做重活/发送，links2004 库对此支持很差）
static uint8_t *current_buffer = NULL;
volatile bool frameReady = false;

static uint16_t *blockBuf = NULL;   // JPEG 块搬运缓冲（DMA 内存）
#define BLOCK_BUF_PX  1024          // 最多 1024 像素/块（MCU 块最大 16x16=256 像素，留余量）

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  static uint32_t outBlocks = 0;
  outBlocks++;
  if (outBlocks == 1) {
    Serial.printf("[OUT] first block x=%d y=%d w=%u h=%u\n", x, y, w, h);
  }
  if (y >= LCD_H || x >= LCD_W) return false;
  if (x + w > LCD_W) w = LCD_W - x;
  if (y + h > LCD_H) h = LCD_H - y;
  if (w == 0 || h == 0) return true;

  if (!blockBuf) {
    blockBuf = (uint16_t *)heap_caps_malloc(BLOCK_BUF_PX * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!blockBuf) { Serial.println("[E] blockBuf alloc failed"); return false; }
  }

  if ((uint32_t)w * h <= BLOCK_BUF_PX) {
    memcpy(blockBuf, bitmap, (uint32_t)w * h * 2);   // 拷到 DMA 内存再发
    esp_lcd_panel_draw_bitmap(lcd_panel, x, y, x + w, y + h, blockBuf);
  } else {
    // 超大块兜底：逐行发送
    for (int row = 0; row < h; row++) {
      memcpy(blockBuf, bitmap + (uint32_t)row * w, (uint32_t)w * 2);
      esp_lcd_panel_draw_bitmap(lcd_panel, x, y + row, x + w, y + row + 1, blockBuf);
    }
  }
  return true;
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Connection is lost!\n", num);
      expected_jpeg_size = 0;
      jpeg_buffer_pos = 0;
      frameReady = false;
      break;

    case WStype_TEXT: {
      Serial.printf("[WS-TEXT] len=%u: %s\n", (unsigned)length, (char*)payload);
      String text = String((char*)payload);
      if (text.startsWith("JPEG_FRAME_SIZE:")) {
        expected_jpeg_size = text.substring(16).toInt();
        jpeg_buffer_pos = 0;
        if (expected_jpeg_size > JPEG_BUFFER_SIZE) {
          Serial.println("Error: JPEG size too large, resetting!");
          expected_jpeg_size = 0;
          jpeg_buffer_pos = 0;
          return;
        }
      }
      break;
    }

    case WStype_BIN: {
      static uint32_t binBytes = 0;
      static uint32_t binMsgs = 0;
      binBytes += length;
      binMsgs++;
      if (binMsgs == 1 || binMsgs % 20 == 0) {
        Serial.printf("[WS-BIN] #%lu msg len=%u, total=%lu KB\n",
                      (unsigned long)binMsgs, (unsigned)length, (unsigned long)(binBytes / 1024));
      }
      if (expected_jpeg_size == 0) {
        Serial.println("Error: No JPEG size received, ignoring data!");
        return;
      }
      if (jpeg_buffer_pos + length <= expected_jpeg_size && jpeg_buffer_pos + length <= JPEG_BUFFER_SIZE) {
        memcpy(current_buffer + jpeg_buffer_pos, payload, length);
        jpeg_buffer_pos += length;

        if (jpeg_buffer_pos >= expected_jpeg_size) {
          // 帧收齐：只置标志，解码/绘制/回执由 websocketTask 主循环完成
          frameReady = true;
        }
      } else {
        Serial.println("Error: Buffer overflow or invalid data, resetting!");
        expected_jpeg_size = 0;
        jpeg_buffer_pos = 0;
      }
      break;
    }

    case WStype_CONNECTED:
      Serial.printf("[%u] Connection established! (clients=%d)\n", num, webSocket.connectedClients());
      break;
  }
}

// 触摸数据由 websocketTask 统一发送（webSocket 库不是线程安全的，
// loop 任务和 websocket 任务并发调用 broadcastTXT 会破坏连接状态）
volatile bool touchPending = false;
volatile uint16_t touchX = 0, touchY = 0;

// JRESULT -> 可读文本（tjpgd 错误码）
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

void websocketTask(void *pvParameters) {
  current_buffer = jpeg_buffer1;
  // 心跳也放到本任务：所有 webSocket 库调用收敛到同一任务，彻底避免并发
  static uint32_t lastHb = 0;
  while (1) {
    // 发送待上报的触摸数据（与 webSocket.loop() 同一任务上下文，安全）
    if (touchPending) {
      touchPending = false;
      char buf[24];
      snprintf(buf, sizeof(buf), "TOUCH:%u,%u", touchX, touchY);
      webSocket.broadcastTXT(buf);
      Serial.printf("[TOUCH] %u, %u\n", touchX, touchY);
    }
    webSocket.loop();

    // 解码+绘制（在回调之外执行；绑核 1，与验证可行的 lcd_test2 相同核心）
    if (frameReady) {
      frameReady = false;
      uint32_t start_time = micros();
      Serial.printf("[DEC] decoding %u bytes, stack HW:%u\n",
                    (unsigned)expected_jpeg_size, (unsigned)uxTaskGetStackHighWaterMark(NULL));
      JRESULT r = TJpgDec.drawJpg(0, 0, current_buffer, expected_jpeg_size);
      uint32_t dt = micros() - start_time;
      frame_count++;
      Serial.printf("[DEC] #%lu result=%s in %lu us, stack HW:%u\n",
                    (unsigned long)frame_count, jresult_str(r),
                    (unsigned long)dt, (unsigned)uxTaskGetStackHighWaterMark(NULL));
      webSocket.broadcastTXT("FRAME_DONE");
      expected_jpeg_size = 0;
      jpeg_buffer_pos = 0;
      current_buffer = (current_buffer == jpeg_buffer1) ? jpeg_buffer2 : jpeg_buffer1;
    }

    // 30 秒心跳日志：证明程序活着，并给出运行状态
    if (millis() - lastHb > 30000) {
      lastHb = millis();
      Serial.printf("[HB] alive | WiFi:%s | clients:%d | frames:%lu | heap:%u KB | psram:%u KB\n",
                    (WiFi.status() == WL_CONNECTED) ? "OK" : "DOWN",
                    webSocket.connectedClients(),
                    (unsigned long)frame_count,
                    ESP.getFreeHeap() / 1024,
                    ESP.getFreePsram() / 1024);
    }
    vTaskDelay(1);
  }
}

// ---------- LCD 初始化（复刻 LuatOS QLCD） ----------
bool lcd_init() {
  pinMode(PIN_LCD_BL, OUTPUT);
  pinMode(40, OUTPUT);
  digitalWrite(PIN_LCD_BL, HIGH);   // 背光先开，报错也能看见
  digitalWrite(40, HIGH);   //ACC

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
    .spi_mode = 3,                  // ST7789: CPOL=1, CPHA=1
    .pclk_hz = 40 * 1000 * 1000,
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

  // 与 devices.lua 相同的方向/反相配置
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
  delay(1500);   // 给 USB CDC 端口留出枚举时间，否则最早的打印会丢失
  Serial.println("\n[BOOT] v3 starting (esp_lcd display)...");
  Serial.printf("[BOOT] SDK: %s\n", ESP.getSdkVersion());

  spiMutex = xSemaphoreCreateMutex();
  if (spiMutex == NULL) {
    Serial.println("Error: Failed to create SPI mutex!");
    while (1) delay(100);
  }

  jpeg_buffer1 = (uint8_t*)ps_malloc(JPEG_BUFFER_SIZE);
  jpeg_buffer2 = (uint8_t*)ps_malloc(JPEG_BUFFER_SIZE);
  if (jpeg_buffer1 == NULL || jpeg_buffer2 == NULL) {
    Serial.println("Error: Failed to allocate JPEG buffers in PSRAM!");
    Serial.printf("[BOOT] PSRAM size: %u KB\n", ESP.getPsramSize() / 1024);
    while (1) delay(100);
  }
  Serial.printf("[BOOT] PSRAM OK: %u KB total, buffers 2x%u KB\n",
                ESP.getPsramSize() / 1024, JPEG_BUFFER_SIZE / 1024);
  current_buffer = jpeg_buffer1;

  if (!lcd_init()) {
    Serial.println("Error: LCD init failed!");
    while (1) delay(100);
  }

  fillScreen(COLOR_BLACK);
  drawText(10, 10, "Booting...", COLOR_WHITE, COLOR_BLACK, 2);

  // 初始化 FT6336U 触摸（I2C，电容触摸无需校准）
  pinMode(TOUCH_RST, OUTPUT);
  digitalWrite(TOUCH_RST, LOW);
  delay(10);
  digitalWrite(TOUCH_RST, HIGH);
  delay(50);

  pinMode(TOUCH_INT, INPUT);   // 可选：低电平表示有触点

  Wire.begin(TOUCH_SDA, TOUCH_SCL);
  Wire.setClock(400000);
  // 探测触摸芯片，方便排查接线
  Wire.beginTransmission(FT6336_ADDR);
  if (Wire.endTransmission() == 0) {
    Serial.println("[TOUCH] FT6336U found on I2C bus.");
  } else {
    Serial.println("[TOUCH] Warning: FT6336U not found! Check SDA=11/SCL=9 wiring.");
  }

  TJpgDec.setJpgScale(1);
  TJpgDec.setSwapBytes(true);
  TJpgDec.setCallback(tft_output);

  // 连接 WiFi（30 秒超时）
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
      while (1) delay(100);   // 停在这里，屏幕和串口都有明确报错
    }
  }
  Serial.printf("\n[WIFI] Connected! IP: %s, RSSI: %d dBm\n",
                WiFi.localIP().toString().c_str(), WiFi.RSSI());

  fillScreen(COLOR_BLACK);
  drawText(10, 10, "WiFi OK!", COLOR_GREEN, COLOR_BLACK, 2);
  drawText(10, 40, WiFi.localIP().toString().c_str(), COLOR_WHITE, COLOR_BLACK, 2);
  drawText(10, 70, "WS port 81 ready", COLOR_WHITE, COLOR_BLACK, 2);

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.printf("[WS] WebSocket server started on port %d\n", websocket_port);

  xTaskCreatePinnedToCore(websocketTask, "WebSocketTask", 16384, NULL, 2, NULL, 1);
  Serial.println("[BOOT] Setup done, entering loop.");
}

void loop() {
  // 触摸读取：只记录，发送由 websocketTask 完成（避免跨任务并发调用 webSocket）
  uint16_t tx, ty;
  if (readTouchScreen(&tx, &ty)) {
    touchX = tx;
    touchY = ty;
    touchPending = true;
  }

  // WiFi 断线检测（自动重连已开启，这里打日志提醒）
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
    xSemaphoreTake(spiMutex, portMAX_DELAY);
    fillScreen(COLOR_BLACK);
    drawText(10, 10, WiFi.localIP().toString().c_str(), COLOR_WHITE, COLOR_BLACK, 2);
    xSemaphoreGive(spiMutex);
  }
  wifiWasUp = wifiUp;

  delay(10);   // 避免刷屏式发送触摸数据
}
