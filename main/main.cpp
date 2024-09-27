#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define TAG "IMU"
#define RAD_TO_DEG (180.0/M_PI)

MPU6050 mpu;
uint8_t fifoBuffer[64];  // FIFO storage buffer
Quaternion q;            // quaternion container
VectorFloat gravity;      // gravity vector
float ypr[3];             // yaw/pitch/roll

int64_t prevTime = 0;
int64_t cycleTime =0;
int64_t currentTime = esp_timer_get_time();

void getYawPitchRoll() {
    
    prevTime = currentTime;
    currentTime = esp_timer_get_time();
    cycleTime = currentTime - prevTime;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    // ESP_LOGI(TAG, "Yaw: %f, Pitch: %f, Roll: %f", ypr[0] * RAD_TO_DEG, ypr[1] * RAD_TO_DEG, ypr[2] * RAD_TO_DEG);

    // ESP_LOGI(TAG, "Yaw: %f, Pitch: %f, Roll: %f (Elapsed Time: %lld us)", ypr[0] * RAD_TO_DEG, ypr[1] * RAD_TO_DEG, ypr[2] * RAD_TO_DEG, esp_timer_get_time());
    printf("%f,%f,%f,%lld\n", ypr[0] * RAD_TO_DEG, ypr[1] * RAD_TO_DEG, ypr[2] * RAD_TO_DEG,cycleTime);

}
void imu_task(void *pvParameters) {
    mpu.initialize();
    uint8_t buffer[1];
    I2Cdev::readByte(MPU6050_ADDRESS_AD0_LOW, MPU6050_RA_WHO_AM_I, buffer);
    ESP_LOGI(TAG, "Device ID: 0x%x", buffer[0]);

    uint8_t devStatus = mpu.dmpInitialize();
    if (devStatus != 0) {
        ESP_LOGE(TAG, "DMP Initialization failed with status: %d", devStatus);
        vTaskDelete(NULL);
    }

    mpu.setDMPEnabled(true);

    while (1) {
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            getYawPitchRoll();
        }
        // vTaskDelay(1 / portTICK_PERIOD_MS);  // Match DMP refresh rate (~10Hz)
    }
}
extern "C" void app_main(void) {
    // Initialize I2C
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GPIO_NUM_15;
    conf.scl_io_num = GPIO_NUM_16;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    conf.clk_flags = 0;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    // Start IMU task
    xTaskCreate(&imu_task, "IMU Task", 2048, NULL, 5, NULL);
}
