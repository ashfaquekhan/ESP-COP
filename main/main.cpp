#include <stdio.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "MadgwickAHRS.h"
#include <driver/gpio.h>
#include <esp_timer.h>
#include <driver/ledc.h>

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

double current_time, last_time;
#define LOOP_RATE_MS 1  // Desired loop rate (e.g., 100 ms)
static esp_timer_handle_t timer; // Timer handle


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

void IRAM_ATTR timer_callback(void* arg) {
    // Get scaled accelerometer and gyroscope values
    _getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Calculate delta time since last update
    current_time = TimeToSec(); // Ensure this returns time in seconds
    dt = current_time - last_time;
    last_time = current_time;

    // Update Madgwick filter with new data
    madgwick.updateIMU(gx, gy, gz, ax, ay, az, dt);
    roll = madgwick.getRoll();
    pitch = madgwick.getPitch();
    // yaw = gz;
    yaw   = madgwick.getYaw();
}

void mpu6050_task(void *pvParameters) {
    // Initialize the MPU6050
    mpu.initialize();
    ESP_LOGI(TAG, "MPU6050 initialized, DeviceID=0x%x", mpu.getDeviceID());

    const esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,          // Callback function
        .arg = NULL,                          // Argument passed to the callback (can be NULL if not needed)
        .dispatch_method = ESP_TIMER_TASK,    // Dispatch method
        .name = "IMU Timer",                  // Name of the timer
        .skip_unhandled_events = false         // Skip unhandled events (set to true if needed)
    };

    esp_err_t err = esp_timer_create(&timer_args, &timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create timer: %s", esp_err_to_name(err));
        return; // Handle error appropriately
    }

    uint64_t timer_interval = LOOP_RATE_MS * 1000; 
    esp_timer_start_periodic(timer, timer_interval);

    while (1) 
    {
         vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    // Cleanup 
    esp_timer_stop(timer);
    esp_timer_delete(timer);
}

// IMU task
void mpu6050_task_direct(void *pvParameters) {
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
        // yaw   = gz;
        yaw   = madgwick.getYaw();
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

void print_task(void *pvParameters)
{
    while (1)
    {
        // printf("Yaw: %f, Pitch: %f, Roll: %f, dt: %f\n", yaw, pitch, roll, dt);
        // printf("%f,%f,%f,%f\n", roll, pitch, yaw,dt);
        printf("%f,%f\n", roll, pitch);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// Function declarations from wifi_webserver.cpp
extern "C" void wifi_webserver_task(void* pvParameters);

// Main application, called from a specific core
extern "C" void app_main(void) 
{
    init_i2c();

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    xTaskCreatePinnedToCore(&mpu6050_task, "mpu6050_task", 1024 * 8, NULL, 5, NULL, 0);
    // xTaskCreatePinnedToCore(&mpu6050_task_direct, "mpu6050_task_direct", 1024 * 8, NULL, 5, NULL, 0);

    xTaskCreatePinnedToCore(print_task, "print_task", 4096, NULL, 5, NULL, 1);

    xTaskCreatePinnedToCore(wifi_webserver_task, "wifi_webserver_task", 4096, NULL, 5, NULL, 1);
}
