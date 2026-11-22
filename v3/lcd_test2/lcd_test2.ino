// LCD 测试 v2：1:1 复刻 LuatOS QLCD 已验证可行的初始化方案
// 参考: D:/Workspace/LUATOS_NG/LuatOS/components/lcd_panel/qlcd.c (qlcd_init)
// 参数完全一致: SPI3_HOST, mode3, 40MHz, 8bit cmd/param,
//              RGB 顺序, invert_color=true, swap_xy=true, mirror(false,true)
// 引脚: MOSI=17 SCK=18 DC=15 RST=7 CS=6 BL=8
#include <Arduino.h>
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#define LCD_W 320
#define LCD_H 240

#define PIN_MOSI 17
#define PIN_CLK  18
#define PIN_DC   15
#define PIN_RST  7
#define PIN_CS   6
#define PIN_BL   8

static esp_lcd_panel_handle_t panel = NULL;

// 整屏填充（逐行发送，只需一行缓冲，DMA 内存）
static void fillScreen(uint16_t color) {
  static uint16_t *line = NULL;
  if (line == NULL) {
    line = (uint16_t *)heap_caps_malloc(LCD_W * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (line == NULL) { Serial.println("[E] line buf alloc failed"); return; }
  }
  for (int i = 0; i < LCD_W; i++) line[i] = color;
  for (int y = 0; y < LCD_H; y++) {
    esp_lcd_panel_draw_bitmap(panel, 0, y, LCD_W, y + 1, line);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n[TEST2] esp_lcd ST7789 test (LuatOS QLCD replica)");

  // 背光（这里手动开，不走库）
  pinMode(PIN_BL, OUTPUT);
  digitalWrite(PIN_BL, HIGH);
  Serial.println("[TEST2] Backlight ON");

  // ---- SPI 总线：SPI3_HOST，与 LuatOS 默认一致 ----
  spi_bus_config_t buscfg = {
    .mosi_io_num = PIN_MOSI,
    .miso_io_num = -1,
    .sclk_io_num = PIN_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = LCD_W * 2,
    .flags = SPICOMMON_BUSFLAG_MASTER,
  };
  esp_err_t ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
  Serial.printf("[TEST2] spi_bus_initialize: %s\n", esp_err_to_name(ret));

  // ---- Panel IO：mode 3, 40MHz, 8bit —— 与 qlcd.c 一致 ----
  esp_lcd_panel_io_handle_t io = NULL;
  esp_lcd_panel_io_spi_config_t io_config = {
    .cs_gpio_num = PIN_CS,
    .dc_gpio_num = PIN_DC,
    .spi_mode = 3,
    .pclk_hz = 40 * 1000 * 1000,
    .trans_queue_depth = 10,
    .on_color_trans_done = NULL,
    .user_ctx = NULL,
    .lcd_cmd_bits = 8,
    .lcd_param_bits = 8,
  };
  ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &io_config, &io);
  Serial.printf("[TEST2] new_panel_io_spi: %s\n", esp_err_to_name(ret));

  // ---- ST7789 面板：RGB 顺序（= 原 ESP_LCD_COLOR_SPACE_RGB），16bpp ----
  esp_lcd_panel_dev_config_t panel_config = {
    .reset_gpio_num = PIN_RST,
    .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
    .bits_per_pixel = 16,
  };
  ret = esp_lcd_new_panel_st7789(io, &panel_config, &panel);
  Serial.printf("[TEST2] new_panel_st7789: %s\n", esp_err_to_name(ret));

  ret = esp_lcd_panel_reset(panel);
  Serial.printf("[TEST2] panel_reset: %s\n", esp_err_to_name(ret));
  ret = esp_lcd_panel_init(panel);
  Serial.printf("[TEST2] panel_init: %s\n", esp_err_to_name(ret));

  // ---- 与 devices.lua 相同的方向/反相配置 ----
  esp_lcd_panel_invert_color(panel, true);   // invert_color = true
  esp_lcd_panel_swap_xy(panel, true);        // swap_xy = true
  esp_lcd_panel_mirror(panel, false, true);  // mirror_x=false, mirror_y=true
  esp_lcd_panel_disp_on_off(panel, true);    // 显示开启
  delay(100);
  Serial.println("[TEST2] Panel initialized (LuatOS QLCD style)");

  // ---- 颜色循环测试 ----
  Serial.println("[TEST2] Filling RED");
  fillScreen(0xF800); delay(2000);
  Serial.println("[TEST2] Filling GREEN");
  fillScreen(0x07E0); delay(2000);
  Serial.println("[TEST2] Filling BLUE");
  fillScreen(0x001F); delay(2000);
  Serial.println("[TEST2] Filling BLACK");
  fillScreen(0x0000); delay(2000);
  Serial.println("[TEST2] Filling WHITE");
  fillScreen(0xFFFF); delay(2000);
  Serial.println("[TEST2] Cycle done, looping...");
}

void loop() {
  fillScreen(0xF800); delay(1500);  // 红
  fillScreen(0x07E0); delay(1500);  // 绿
  fillScreen(0x001F); delay(1500);  // 蓝
}
