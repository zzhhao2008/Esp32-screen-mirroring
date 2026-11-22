// LCD 连通性测试：用于区分"颜色反相"和"SPI 不通"两种白屏原因
// 使用与 v3 相同的 TFT_eSPI 配置（ST7789_S3.h）
// 烧录后观察屏幕并对照串口输出的判断表
#include <TFT_eSPI.h>
#include <SPI.h>

TFT_eSPI tft = TFT_eSPI();

void showColor(uint16_t color, const char* name) {
  tft.fillScreen(color);
  Serial.printf("[TEST] fillScreen %s (0x%04X)\n", name, color);
  delay(2000);
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n[TEST] LCD test starting...");
  Serial.printf("[TEST] Screen size: %dx%d\n", tft.width(), tft.height());

  tft.init();
  tft.setRotation(1);
  Serial.println("[TEST] tft.init() done (backlight should be ON now)");

  // 白底黑字：如果反相是问题，这里会显示成黑底白字，文字可读！
  tft.fillScreen(TFT_WHITE);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("If you can read");
  tft.println("this text clearly,");
  tft.println("inversion was the");
  tft.println("problem!");
  delay(4000);

  // 依次刷三原色，每个停 2 秒
  showColor(TFT_RED,   "RED");
  showColor(TFT_GREEN, "GREEN");
  showColor(TFT_BLUE,  "BLUE");
  showColor(TFT_BLACK, "BLACK");
  showColor(TFT_WHITE, "WHITE");

  Serial.println("[TEST] Cycle done, looping...");
}

void loop() {
  showColor(TFT_RED,   "RED");
  showColor(TFT_GREEN, "GREEN");
  showColor(TFT_BLUE,  "BLUE");
}
