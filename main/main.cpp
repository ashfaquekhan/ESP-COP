#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "MadgwickAHRS.h"
#include <driver/gpio.h>

#define RAD_TO_DEG (180.0 / M_PI)

static const char *TAG = "IMU";

// MPU6050 and Madgwick filter objects
MPU6050 mpu;
Madgwick madgwick;

// Sensitivity values
float accel_sensitivity = 16384.0;
float gyro_sensitivity = 131.0;

float roll;
float pitch;
float yaw;  
float dt;
float ax, ay, az, gx, gy, gz;

// Function to get scaled accelerometer and gyroscope data
void _getMotion6(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;

    // Read raw accelerometer and gyroscope data from the MPU6050
    mpu.getMotion6(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);

    // Scale the raw values
    *ax = (float)accel_x / accel_sensitivity;
    *ay = (float)accel_y / accel_sensitivity;
    *az = (float)accel_z / accel_sensitivity;

    *gx = (float)gyro_x / gyro_sensitivity;
    *gy = (float)gyro_y / gyro_sensitivity;
    *gz = (float)gyro_z / gyro_sensitivity;
}

// Function to get time in seconds
double TimeToSec() {
    int64_t time_us = esp_timer_get_time();
    return (double)time_us / 1000000.0;
}

// IMU task
void mpu6050_task(void *pvParameters) {
    // Initialize the MPU6050
    mpu.initialize();
    ESP_LOGI(TAG, "MPU6050 initialized, DeviceID=0x%x", mpu.getDeviceID());

    // Variables for timing and initialization
    double last_time = TimeToSec();
     esp_rom_gpio_pad_select_gpio(GPIO_NUM_11);
    gpio_set_direction(GPIO_NUM_11, GPIO_MODE_OUTPUT);


    while (1) {
        
        // gpio_set_level(GPIO_NUM_11, 1); // Turn on the LED
        // Get scaled accelerometer and gyroscope values
        _getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        // Calculate delta time since last update
        dt = TimeToSec() - last_time;
        last_time = TimeToSec();

        // Update Madgwick filter with new data
        madgwick.updateIMU(gx, gy, gz, ax, ay, az, dt);
        roll  = madgwick.getRoll();
        pitch = madgwick.getPitch();
        yaw   = madgwick.getYaw();
        // ESP_LOGI(TAG, "Roll: %f, Pitch: %f, Yaw: %f,%lld",roll,pitch,yaw,cycleTime);
        // printf("%f,%f,%f,%f\n",roll,pitch,yaw,dt);
        // gpio_set_level(GPIO_NUM_11, 0); // Turn on the LED
    }
}

// I2C initialization
void init_i2c(void) {
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
}

// Main function
extern "C" void app_main(void) {
    // Initialize I2C
    init_i2c();

    // Create the IMU task
    xTaskCreate(&mpu6050_task, "mpu6050_task", 1024 * 8, NULL, 5, NULL);

}
