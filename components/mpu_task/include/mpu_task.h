
#ifndef _MPU_TASK_
#define _MPU_TASK_

#include <iostream>

// #include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "I2Cbus.h"
#include "MPU.h"
#include "mpu/math.h"
#include "mpu/types.h"


#define FIFO_STREAM_SIZE 128
int mpu_init();

#endif