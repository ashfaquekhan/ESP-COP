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
#include "driver/adc.h"
#include "esp_adc_cal.h"


#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD 0.0174533

#define CONSTRAIN(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

#define LOW_PASS_FILTER(input, output, prev_output, alpha) \
    do { \
        (output) = (alpha) * (input) + (1.0 - (alpha)) * (prev_output); \
        (prev_output) = (output); \
    } while (0)

#define TIME_BASED_LOW_PASS_FILTER(input, output, prev_output, T, period) \
    { \
        (output) = ((T) / ((T) + (period))) * (prev_output) + ((period) / ((T) + (period))) * (input); \
        (prev_output) = (output); \
    }


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

float alpha(0.006); //0.015~0.035
// float alphaAcc(0.09);
float period(0.001);
float tKf(0.003);
float alphaM = 0.1;
// float dtC = 0.0001;    // Time step (in seconds)
double current_time, last_time;

#define LOOP_RATE_MS 1  // Desired loop rate in MS(e.g., 100 ms)
static esp_timer_handle_t timer; // Timer handle

// Define the ADC width and attenuation
#define ADC_WIDTH ADC_WIDTH_BIT_12  // 12-bit resolution: values range from 0 to 4095
#define ADC_ATTEN ADC_ATTEN_DB_12   // Attenuation 11dB: allows reading up to 3.6V
#define ADC_CHANNEL ADC1_CHANNEL_8  // Change this based on the pin you use
int adc_value;
float volt;
bool voltInit=false;
int battCount;


// State and covariance matrices
float x[6] = {0}; // [roll, pitch, yaw, gyro_bias_x, gyro_bias_y, gyro_bias_z]
float P[6][6] = {0}; // Covariance matrix

// Process noise and measurement noise
const float Q[6][6] = {
    {1e-3, 0, 0, 0, 0, 0},
    {0, 1e-3, 0, 0, 0, 0},
    {0, 0, 1e-3, 0, 0, 0},
    {0, 0, 0, 1e-5, 0, 0},
    {0, 0, 0, 0, 1e-5, 0},
    {0, 0, 0, 0, 0, 1e-5}
};

const float R[3][3] = {
    {1e-2, 0, 0},
    {0, 1e-2, 0},
    {0, 0, 1e-2}
};

// Predict step
void EKF_Predict(float gx, float gy, float gz, float dt) {
    // Predict state using the gyroscope
    x[0] += (gx - x[3]) * dt; // Roll
    x[1] += (gy - x[4]) * dt; // Pitch
    x[2] += (gz - x[5]) * dt; // Yaw

    // Predict covariance
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            P[i][j] += Q[i][j] * dt;
        }
    }
}

// Update step
void EKF_Update(float ax, float ay, float az) {
    // Calculate expected measurements
    float roll_meas = atan2(ay, az);
    float pitch_meas = atan2(-ax, sqrt(ay * ay + az * az));

    float y[2] = {roll_meas - x[0], pitch_meas - x[1]}; // Measurement residual

    // Compute Kalman gain
    float S[2][2] = {0};
    float K[6][2] = {0};
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            S[i][j] = R[i][j];
            for (int k = 0; k < 6; k++) {
                S[i][j] += P[k][i] * P[k][j];
            }
        }
    }

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++) {
                K[i][j] += P[i][k] * S[k][j];
            }
        }
    }

    // Update state
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 2; j++) {
            x[i] += K[i][j] * y[j];
        }
    }

    // Update covariance
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            for (int k = 0; k < 2; k++) {
                P[i][j] -= K[i][k] * P[k][j];
            }
        }
    }
}


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

void taskfunc()
{
        dt = TimeToSec() - last_time;
        last_time = TimeToSec();
        // gpio_set_level(GPIO_NUM_11, 1); // Turn on the LED
        // Get scaled accelerometer and gyroscope values
        _getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // // // Update Madgwick filter with new data
        // madgwick.updateIMU(gx, gy, gz, ax, ay, az, dt);
        // roll  = madgwick.getRoll();
        // pitch = madgwick.getPitch();
        // // yaw   = gz;
        // yaw   = madgwick.getYaw();

        // // EKF Predict
        // EKF_Predict(gx, gy, gz, dt);

        // // EKF Update
        // EKF_Update(ax, ay, az);

        // Retrieve filtered states
        roll = x[0];
        pitch = x[1];
        yaw = x[2];
        

}

void IRAM_ATTR timer_callback(void* arg) 
{
    taskfunc();
}

void mpu6050_task(void *pvParameters) {
    // Initialize the MPU6050
    mpu.initialize();
    mpu.setXAccelOffset(-707);
    mpu.setYAccelOffset(696);
    mpu.setZAccelOffset(1106);

    mpu.setXGyroOffset(118);
    mpu.setYGyroOffset(-31);
    mpu.setZGyroOffset(62);

//-------------------------------------------
	// Get DeviceID
	uint8_t devid = mpu.getDeviceID();
	ESP_LOGI(TAG, "devid=0x%x", devid);

	// Get the sample rate
	ESP_LOGI(TAG, "getRate()=%d", mpu.getRate());
	// Set the sample rate to 8kHz
	if (mpu.getRate() != 0) mpu.setRate(0);

	// Get FSYNC configuration value
	ESP_LOGI(TAG, "getExternalFrameSync()=%d", mpu.getExternalFrameSync());
	// Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering
	if (mpu.getExternalFrameSync() != 0) mpu.setExternalFrameSync(0);

	// Set Digital Low Pass Filter
	ESP_LOGI(TAG, "getDLPFMode()=%d", mpu.getDLPFMode());
	if (mpu.getDLPFMode() != 6) mpu.setDLPFMode(6);

	// Get Accelerometer Scale Range
	ESP_LOGI(TAG, "getFullScaleAccelRange()=%d", mpu.getFullScaleAccelRange());
	// Set Accelerometer Full Scale Range to ±2g
	if (mpu.getFullScaleAccelRange() != 0) mpu.setFullScaleAccelRange(0); // -2 --> +2[g]
	accel_sensitivity = 16384.0; // g

	// Get Gyro Scale Range
	ESP_LOGI(TAG, "getFullScaleGyroRange()=%d", mpu.getFullScaleGyroRange());
	// Set Gyro Full Scale Range to ±250deg/s
	if (mpu.getFullScaleGyroRange() != 0) mpu.setFullScaleGyroRange(0); // -250 --> +250[Deg/Sec]
	gyro_sensitivity = 131.0; // Deg/Sec
//-------------------------------------------


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

    mpu.setXAccelOffset(-707);
    mpu.setYAccelOffset(696);
    mpu.setZAccelOffset(1106);

    mpu.setXGyroOffset(118);
    mpu.setYGyroOffset(-31);
    mpu.setZGyroOffset(62);

    ESP_LOGI(TAG, "MPU6050 initialized, DeviceID=0x%x", mpu.getDeviceID());

    // esp_rom_gpio_pad_select_gpio(GPIO_NUM_11);
    // gpio_set_direction(GPIO_NUM_11, GPIO_MODE_OUTPUT);
    while (1) 
    {
        taskfunc();
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
        adc_value = adc1_get_raw(ADC_CHANNEL);
        volt =  volt*0.999  +  adc_value *(0.0008791)* 0.001;
        if(!voltInit)
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
            volt = adc_value *(0.0008791);
            voltInit=true;
        }
        if(voltInit && volt<1.5)
        {
            battCount++;
        }
        if(battCount>100)
        {
        }
        else
        {
            battCount=0;
        }
        // printf("ADC Value: %d Volt: %.2f\n", adc_value,volt);
        // printf("Yaw: %f, Pitch: %f, Roll: %f, dt: %f\n", yaw, pitch, roll, dt);
        // printf("%.2f,%.2f,%.2f\n",fax,fay,faz);
        printf("%.2f,%.2f,%.2f\n",pitch,roll,yaw);
        // printf("%.2f,%.2f,%.2f,%.2f\n",rKd*fdR,(rKi*iR + errR*rKp),gx,roll);
        //  printf("%.2f,%.2f\n",dP,fdP);
        //  printf("%.2f,%.2f\n",dR,fdR);
        // printf("%d,%.2f,%.2f,%.2f\n",throt,Rin,Pin,Yin);
        vTaskDelay(10/ portTICK_PERIOD_MS);
    }
}

extern "C" void app_main(void) 
{
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);
    
    init_i2c();

 
    xTaskCreatePinnedToCore(&mpu6050_task, "mpu6050_task", 1024 * 8, NULL, 5, NULL, 0);
    // xTaskCreatePinnedToCore(&mpu6050_task_direct, "mpu6050_task_direct", 1024 * 8, NULL, 5, NULL, 0);

    xTaskCreatePinnedToCore(print_task, "print_task", 4096, NULL, 5, NULL, 1);

}

