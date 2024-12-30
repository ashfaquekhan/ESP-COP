```cpp
#include <stdio.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "imu.h"
#include "motor_control.h"
#include "flight_controller.h"

static const char *TAG = "DRONE";

#define ADC_WIDTH ADC_WIDTH_BIT_12
#define ADC_ATTEN ADC_ATTEN_DB_12
#define ADC_CHANNEL ADC1_CHANNEL_8
#define VOLTAGE_THRESHOLD 1.5
#define BATTERY_COUNT_THRESHOLD 100

static IMU imu;
static MotorController motors;
static FlightController* flightController;
static float voltage = 0;
static bool voltageInitialized = false;
static int batteryCount = 0;

void initADC() {
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);
}

void batteryMonitorTask(void* pvParameters) {
    while (1) {
        int adcValue = adc1_get_raw(ADC_CHANNEL);
        voltage = voltage * 0.999 + adcValue * (0.0008791) * 0.001;

        if (!voltageInitialized) {
            vTaskDelay(pdMS_TO_TICKS(100));
            voltage = adcValue * (0.0008791);
            voltageInitialized = true;
        }

        if (voltageInitialized && voltage < VOLTAGE_THRESHOLD) {
            batteryCount++;
        }

        if (batteryCount > BATTERY_COUNT_THRESHOLD) {
            motors.setEnabled(false);
        } else {
            batteryCount = 0;
        }

        ESP_LOGI(TAG, "Battery Voltage: %.2fV", voltage);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void flightControlTask(void* pvParameters) {
    static uint32_t lastTime = 0;
    const uint32_t interval_ms = 1;

    while (1) {
        uint32_t currentTime = esp_timer_get_time() / 1000;
        float dt = (currentTime - lastTime) / 1000000.0f;
        lastTime = currentTime;

        flightController->update(dt);
        vTaskDelay(pdMS_TO_TICKS(interval_ms));
    }
}

void debugPrintTask(void* pvParameters) {
    while (1) {
        ESP_LOGI(TAG, "Roll: %.2f, Pitch: %.2f, Yaw: %.2f",
                 imu.getRoll(), imu.getPitch(), imu.getYaw());
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

extern "C" void wifi_webserver_task(void* pvParameters);

extern "C" void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize subsystems
    initADC();
    imu.init();
    motors.init();
    flightController = new FlightController(imu, motors);
    flightController->init();

    // Create tasks
    xTaskCreatePinnedToCore(flightControlTask, "flight_control", 8192, NULL, 5, NULL, 0);
    // xTaskCreatePinnedToCore(batteryMonitorTask, "battery_monitor", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(debugPrintTask, "debug_print", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(wifi_webserver_task, "wifi_webserver", 4096, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "Drone initialization complete");
}

