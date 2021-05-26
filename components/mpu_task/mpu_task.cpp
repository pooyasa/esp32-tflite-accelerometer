#include "mpu_task.h"

static const char* TAG = "MPU";
static constexpr int kInterruptPin         = 13;  // GPIO_NUM
static constexpr gpio_num_t SDA = GPIO_NUM_16;
static constexpr gpio_num_t SCL = GPIO_NUM_17;
static constexpr uint32_t CLOCK_SPEED = 400000;  // range from 100 KHz ~ 400Hz
static constexpr mpud::int_config_t kInterruptConfig{
    .level = mpud::INT_LVL_ACTIVE_HIGH,
    .drive = mpud::INT_DRV_PUSHPULL,
    .mode  = mpud::INT_MODE_PULSE50US,
    .clear = mpud::INT_CLEAR_STATUS_REG  //
};



static void mpuTask(void*);
static void printTask(void*);
static IRAM_ATTR void mpuISR(TaskHandle_t);


MPU_t MPU;  // create a default MPU object
float roll{0}, pitch{0}, yaw{0};

int mpu_init(){
    printf("$ MPU Driver Example: MPU-I2C\n");
    fflush(stdout);

    // Initialize I2C on port 0 using I2Cbus interface
    i2c0.begin(SDA, SCL, CLOCK_SPEED);


    MPU.setBus(i2c0);  // set bus port, not really needed since default is i2c0
    MPU.setAddr(mpud::MPU_I2CADDRESS_AD0_LOW);  // set address, default is AD0_LOW

    // Great! Let's verify the communication
    // (this also check if the connected MPU supports the implementation of chip selected in the component menu)
    while (esp_err_t err = MPU.testConnection()) {
        ESP_LOGE(TAG, "Failed to connect to the MPU, error=%#X", err);
        return 1;
    }
    ESP_LOGI(TAG, "MPU connection successful!");

    // Initialize
    ESP_ERROR_CHECK(MPU.initialize());  // initialize the chip and set initial configurations
    ESP_ERROR_CHECK(MPU.setSampleRate(25));  // set sample rate to 25 Hz
    ESP_ERROR_CHECK(MPU.setAccelFullScale(mpud::ACCEL_FS_4G));

    xTaskCreate(mpuTask, "mpuTask", 4 * 1024, nullptr, 6, nullptr);
    xTaskCreate(printTask, "printTask", 2 * 1024, nullptr, 5, nullptr);
    return 0;

    // Calibrate the accelerometer
    mpud::raw_axes_t accelBias, gyroBias;
    ESP_ERROR_CHECK(MPU.computeOffsets(&accelBias, &gyroBias));
    ESP_ERROR_CHECK(MPU.setAccelOffset(accelBias));
    ESP_ERROR_CHECK(MPU.setGyroOffset(gyroBias));
    
    // Setup FIFO
    ESP_ERROR_CHECK(MPU.setFIFOConfig(mpud::FIFO_CFG_ACCEL));
    ESP_ERROR_CHECK(MPU.setFIFOEnabled(true));
    constexpr uint16_t kFIFOPacketSize = 12;

    // Setup Interrupt
    constexpr gpio_config_t kGPIOConfig{
        .pin_bit_mask = (uint64_t) 0x1 << kInterruptPin,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_POSEDGE  //
    };
    gpio_config(&kGPIOConfig);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add((gpio_num_t) kInterruptPin, mpuISR, xTaskGetCurrentTaskHandle());
    ESP_ERROR_CHECK(MPU.setInterruptConfig(kInterruptConfig));
    ESP_ERROR_CHECK(MPU.setInterruptEnabled(mpud::INT_EN_RAWDATA_READY));

    // Ready to start reading
    ESP_ERROR_CHECK(MPU.resetFIFO());  // start clean


}

static void mpuTask(void*) {
    // Calibrate the accelerometer
    mpud::raw_axes_t accelBias, gyroBias;
    ESP_ERROR_CHECK(MPU.computeOffsets(&accelBias, &gyroBias));
    ESP_ERROR_CHECK(MPU.setAccelOffset(accelBias));
    ESP_ERROR_CHECK(MPU.setGyroOffset(gyroBias));
    
    // Setup FIFO
    ESP_ERROR_CHECK(MPU.setFIFOConfig(mpud::FIFO_CFG_ACCEL));
    ESP_ERROR_CHECK(MPU.setFIFOEnabled(true));
    constexpr uint16_t kFIFOPacketSize = 12;

    // Setup Interrupt
    constexpr gpio_config_t kGPIOConfig{
        .pin_bit_mask = (uint64_t) 0x1 << kInterruptPin,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_POSEDGE  //
    };
    gpio_config(&kGPIOConfig);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add((gpio_num_t) kInterruptPin, mpuISR, xTaskGetCurrentTaskHandle());
    ESP_ERROR_CHECK(MPU.setInterruptConfig(kInterruptConfig));
    ESP_ERROR_CHECK(MPU.setInterruptEnabled(mpud::INT_EN_RAWDATA_READY));

    // Ready to start reading
    ESP_ERROR_CHECK(MPU.resetFIFO());  // start clean

    while (true) {
        // Wait for notification from mpuISR
        uint32_t notificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (notificationValue > 1) {
            ESP_LOGW(TAG, "Task Notification higher than 1, value: %d", notificationValue);
            MPU.resetFIFO();
            continue;
        }
        // Check FIFO count
        uint16_t fifocount = MPU.getFIFOCount();
        if (esp_err_t err = MPU.lastError()) {
            ESP_LOGE(TAG, "Error reading fifo count, %#X", err);
            MPU.resetFIFO();
            continue;
        }
        if (fifocount > kFIFOPacketSize * 2) {
            if (!(fifocount % kFIFOPacketSize)) {
                ESP_LOGE(TAG, "Sample Rate too high!, not keeping up the pace!, count: %d", fifocount);
            }
            else {
                ESP_LOGE(TAG, "FIFO Count misaligned! Expected: %d, Actual: %d", kFIFOPacketSize, fifocount);
            }
            MPU.resetFIFO();
            continue;
        }
        // Burst read data from FIFO
        uint8_t buffer[kFIFOPacketSize];
        if (esp_err_t err = MPU.readFIFO(kFIFOPacketSize, buffer)) {
            ESP_LOGE(TAG, "Error reading sensor data, %#X", err);
            MPU.resetFIFO();
            continue;
        }
        // Format
        mpud::raw_axes_t rawAccel, rawGyro;
        rawAccel.x = buffer[0] << 8 | buffer[1];
        rawAccel.y = buffer[2] << 8 | buffer[3];
        rawAccel.z = buffer[4] << 8 | buffer[5];
        rawGyro.x  = buffer[6] << 8 | buffer[7];
        rawGyro.y  = buffer[8] << 8 | buffer[9];
        rawGyro.z  = buffer[10] << 8 | buffer[11];
        // Calculate tilt angle
        // range: (roll[-180,180]  pitch[-90,90]  yaw[-180,180])
        constexpr double kRadToDeg = 57.2957795131;
        constexpr float kDeltaTime = 1.f / kSampleRate;
        float gyroRoll             = roll + mpud::math::gyroDegPerSec(rawGyro.x, kGyroFS) * kDeltaTime;
        float gyroPitch            = pitch + mpud::math::gyroDegPerSec(rawGyro.y, kGyroFS) * kDeltaTime;
        float gyroYaw              = yaw + mpud::math::gyroDegPerSec(rawGyro.z, kGyroFS) * kDeltaTime;
        float accelRoll            = atan2(-rawAccel.x, rawAccel.z) * kRadToDeg;
        float accelPitch = atan2(rawAccel.y, sqrt(rawAccel.x * rawAccel.x + rawAccel.z * rawAccel.z)) * kRadToDeg;
        // Fusion
        roll  = gyroRoll * 0.95f + accelRoll * 0.05f;
        pitch = gyroPitch * 0.95f + accelPitch * 0.05f;
        yaw   = gyroYaw;
        // correct yaw
        if (yaw > 180.f)
            yaw -= 360.f;
        else if (yaw < -180.f)
            yaw += 360.f;
    }
    vTaskDelete(nullptr);

}

static void printTask(void*)
{
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    while (true) {
        printf("Pitch: %+6.1f \t Roll: %+6.1f \t Yaw: %+6.1f \n", pitch, roll, yaw);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

static IRAM_ATTR void mpuISR(TaskHandle_t taskHandle)
{
    BaseType_t HPTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(taskHandle, &HPTaskWoken);
    if (HPTaskWoken == pdTRUE) portYIELD_FROM_ISR();
}
