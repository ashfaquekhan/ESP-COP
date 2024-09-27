#include <stdio.h>
#include "driver/i2c.h"
#include "mpu6050.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#define I2C_MASTER_SCL_IO 16      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 15      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 400000 /*!< I2C master clock frequency */

static const char *TAG = "mpu6050_example";
static mpu6050_handle_t mpu6050 = NULL;

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_bus_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL,
    };

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

/**
 * @brief Initialize and configure the MPU6050 sensor
 */
static esp_err_t i2c_sensor_mpu6050_init(void)
{
    esp_err_t ret = i2c_bus_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C bus");
        return ret;
    }

    mpu6050 = mpu6050_create(I2C_MASTER_NUM, MPU6050_I2C_ADDRESS);
    if (mpu6050 == NULL) {
        ESP_LOGE(TAG, "Failed to create MPU6050 handle");
        return ESP_FAIL;
    }

    ret = mpu6050_config(mpu6050, ACCE_FS_8G, GYRO_FS_500DPS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure MPU6050: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = mpu6050_wake_up(mpu6050);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

void app_main(void)
{
    esp_err_t ret;
    uint8_t mpu6050_deviceid;
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    mpu6050_temp_value_t temp;
    complimentary_angle_t comp;

    // Initialize and configure the MPU6050 sensor
    ret = i2c_sensor_mpu6050_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor initialization failed");
        return;
    }

    // Get device ID and verify
    ret = mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);
    if (ret == ESP_OK && mpu6050_deviceid == MPU6050_WHO_AM_I_VAL) {
        ESP_LOGI(TAG, "MPU6050 device ID verified");
    } else {
        ESP_LOGE(TAG, "Failed to verify MPU6050 device ID");
        return;
    }

    // Start continuous reading of sensor data
    while (1) {
        // Get accelerometer data
        ret = mpu6050_get_acce(mpu6050, &acce);
        // if (ret == ESP_OK) {
        //     ESP_LOGI(TAG, "Accelerometer data: X=%.2f, Y=%.2f, Z=%.2f ", acce.acce_x, acce.acce_y, acce.acce_z);
        // } else {
        //     ESP_LOGE(TAG, "Failed to get accelerometer data: %s", esp_err_to_name(ret));
        // }

        // Get gyroscope data
        ret = mpu6050_get_gyro(mpu6050, &gyro);
        // if (ret == ESP_OK) {
        //     ESP_LOGI(TAG, "Gyroscope data: X=%.2f, Y=%.2f, Z=%.2f", gyro.gyro_x, gyro.gyro_y, gyro.gyro_z);
        // } else {
        //     ESP_LOGE(TAG, "Failed to get gyroscope data: %s", esp_err_to_name(ret));
        // }
        // ret = mpu6050_complimentory_filter(mpu6050,&acce,&gyro,&comp);
        // if (ret == ESP_OK)
        // {
            printf("%.2f,%.2f,%.2f\n",gyro.gyro_x,gyro.gyro_y,gyro.gyro_z);
        // }

        // // Get temperature data
        // ret = mpu6050_get_temp(mpu6050, &temp);
        // if (ret == ESP_OK) {
        //     ESP_LOGI(TAG, "Temperature: %.2f°C", temp.temp);
        // } else {
        //     ESP_LOGE(TAG, "Failed to get temperature: %s", esp_err_to_name(ret));
        // }

        // Wait for 1 second before reading again
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // Clean up (not reached because of while(1), but good practice for future modifications)
    mpu6050_delete(mpu6050);
    i2c_driver_delete(I2C_MASTER_NUM);
}
