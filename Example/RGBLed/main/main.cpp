#include "RGBLed.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using namespace DC_Motor_Controller_Firmware::RGB;

extern "C" void app_main() {
  constexpr gpio_num_t RED_PIN = GPIO_NUM_12;
  constexpr gpio_num_t GREEN_PIN = GPIO_NUM_13;
  constexpr gpio_num_t BLUE_PIN = GPIO_NUM_14;

  constexpr ledc_channel_t RED_CH = LEDC_CHANNEL_0;
  constexpr ledc_channel_t GREEN_CH = LEDC_CHANNEL_1;
  constexpr ledc_channel_t BLUE_CH = LEDC_CHANNEL_2;

  constexpr uint16_t DELAY_MS = 1000;
  constexpr uint16_t FADE_DURATION_MS = 800;
  constexpr uint8_t BLINK_DELAY_MS = 200;
  constexpr uint8_t BLINK_COUNT = 2;
  constexpr uint8_t BRIGHTNESS_PERCENT = 60;

  ledIoConfig ledConfig = {
      .pinRed = RED_PIN,
      .pinGreen = GREEN_PIN,
      .pinBlue = BLUE_PIN,
      .redPwmChannel = RED_CH,
      .greenPwmChannel = GREEN_CH,
      .bluePwmChannel = BLUE_CH,
  };

  RGBLed led(ledConfig);

  if (led.init() != ESP_OK) {
    printf("LED init failed!\n");
    return;
  }

  led.setBrightness(BRIGHTNESS_PERCENT);

  const PresetColor colors[] = {PresetColor::RED,  PresetColor::GREEN,
                                PresetColor::BLUE, PresetColor::YELLOW,
                                PresetColor::CYAN, PresetColor::MAGENTA,
                                PresetColor::WHITE};

  printf("LED test started!\n");

  while (true) {
    for (const auto &color : colors) {
      led.setColor(color);
      vTaskDelay(pdMS_TO_TICKS(DELAY_MS));

      led.blinkLed(BLINK_DELAY_MS, BLINK_COUNT);
      vTaskDelay(pdMS_TO_TICKS(DELAY_MS));

      led.fadeToColor(50, 50, 50, FADE_DURATION_MS);
      vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }

    led.turnOffLed();
    vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
  }
}
