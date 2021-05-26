#ifndef _MPU_TASK_
#define _MPU_TASK_


#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "I2Cbus.hpp"
#include "MPU.hpp"
#include "mpu/math.hpp"
#include "mpu/types.hpp"

int mpu_init();

#endif