// =========================================================================
// Released under the MIT License
// Copyright 2017-2018 Natanael Josue Rabello. All rights reserved.
// For the license information refer to LICENSE file in root directory.
// =========================================================================

/**
 * @file mpu_i2c.cpp
 * Example on how to setup MPU through I2C for basic usage.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
using namespace std;

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "mpu_task.h"
#include "main_functions.h"

static const char* TAG = "main";

void tf_main(void) {
    setup();
    while (true) {
        loop();
    }
}
extern "C" void app_main() {
    ESP_LOGI(TAG, "Starting the MPU unit!");
    ESP_ERROR_CHECK(mpu_init());
    ESP_LOGI(TAG, "Starting TFlite unit!");
    xTaskCreate((TaskFunction_t)&tf_main, "tensorflow", 32 * 1024, NULL, 8, NULL);
    vTaskDelete(NULL);
}
