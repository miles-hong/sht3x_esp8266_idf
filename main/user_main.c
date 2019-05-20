/**
 * Simple example with SHT3x sensor. 
 *
 * It shows different user task implementations in *single shot mode* and
 * *periodic mode*. In *single shot* mode either low level or high level
 * functions are used.
 * 
 * Constants SINGLE_SHOT_LOW_LEVEL and SINGLE_SHOT_HIGH_LEVEL controls which
 * task implementation is used.
 *
 * Harware configuration:
 *
 *    +-----------------+     +----------+
 *    | ESP8266 / ESP32 |     | SHT3x    |
 *    |                 |     |          |
 *    |   GPIO 14 (SCL) ------> SCL      |
 *    |   GPIO 13 (SDA) <-----> SDA      |
 *    +-----------------+     +----------+
 */

/* -- use following constants to define the example mode ----------- */

// #define SINGLE_SHOT_LOW_LEVEL
// #define SINGLE_SHOT_HIGH_LEVEL

/* -- includes ----------------------------------------------------- */

#include "sht3x.h"

/* -- platform dependent definitions ------------------------------- */

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "Start SHT3X Example ***************************\n");
    xTaskCreate(I2C_SHT3x_Task, "I2C_SHT3x_Task", 2048, NULL, 10, NULL);
}