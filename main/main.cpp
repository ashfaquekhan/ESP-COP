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
bool clamp = true;

float rKp=0.09,rKi=0.0001,rKd=0.07;
float pKp=0.09,pKi=0.0001,pKd=0.07;
float yKp=0.1,yKi=0.0009,yKd=0.0;

float errR,errP,errY;
float errRprv(0.0),errPprv(0.0),errYprv(0.0);
float iR,iP,iY;
float iRprv(0.0),iPprv(0.0),iYprv(0.0);
float dR,dP,dY;
float fdR,fdP,fdY;
float fdRprv,fdPprv,fdYprv;

float rPID,pPID,yPID;
float rSet,pSet,ySet;
float rOff(3.0),pOff(3.0),yOff;
float iLimit;
int throt = 5; 
float alpha(0.09); //0.015~0.035
float period(0.001);
float tKf(0.003);
bool motrState=false;
int m1,m2,m3,m4;
int m1s,m2s,m3s,m4s;
double current_time, last_time;
int pwmxPID=200;
#define LOOP_RATE_MS 1  // Desired loop rate in MS(e.g., 100 ms)
static esp_timer_handle_t timer; // Timer handle

// LEDC timer configuration
void init_ledc_timer() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,     // High-speed mode
        .duty_resolution = LEDC_TIMER_8_BIT,    // 8-bit resolution
        .timer_num = LEDC_TIMER_2,              // Timer 2
        .freq_hz = 7000,                       // 10 kHz PWM frequency
        .clk_cfg = LEDC_AUTO_CLK                // Auto-select clock
    };
    ledc_timer_config(&ledc_timer);
}

// LEDC channel configuration
void init_ledc_channel(ledc_channel_t channel, int gpio_num) {
    ledc_channel_config_t ledc_channel = {
        .gpio_num = gpio_num,                   // Set GPIO pin
        .speed_mode = LEDC_LOW_SPEED_MODE,     // High-speed mode
        .channel =  channel,                     // Channel
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_2,               // Use Timer 2
        .duty = 0,                              // Set initial duty to 0 (motor off)
        .hpoint = 0,                            // Default
        .flags = {                              // Initialize flags
            .output_invert = 0                  // Disable output inversion (set to 1 to enable)
        }
    };
    ledc_channel_config(&ledc_channel);
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

void initfunc()
{
    pSet=-2;
    rSet=0;
    ySet=0;
    iLimit=50000;
}
void taskfunc()
{
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
        yaw   = gz;
        // yaw   = madgwick.getYaw();

        clamp = throt < 10;
        
        errP = pSet - pitch;
        iP = iPprv + errP;//*dt;
        if(clamp){iP=0;} //clamp
        iP = CONSTRAIN(iP,-iLimit,iLimit);//windup 
        dP= gy;
        // dP = (errP-errPprv)/dt;
        LOW_PASS_FILTER(dP,fdP,fdPprv,alpha);
        pPID = 0.01 * (pKp*errP + pKi*iP - pKd*fdP);         //scale 0.01(scale for 1)*50(max PWM) = 0.5
        iPprv =iP;
        errPprv=errP; 

        errR = rSet + roll;
        iR = iRprv + errR;// *dt;
        if(clamp){iR=0;}
        iR = CONSTRAIN(iR,-iLimit,iLimit);
        dR = -gx;
        // dR = (errR - errRprv)/0.0001;
        LOW_PASS_FILTER(dR,fdR,fdRprv,alpha);
        rPID = 0.01 * (rKp*errR + rKi*iR - rKd*fdR);    //scale 0.01(scale for 1)*50(max PWM) = 0.5
        iRprv = iR;
        errRprv=errR;

        errY = ySet - yaw;
        iY = iYprv + errY;//*dt;
        if(clamp){iY=0;}
        iY = CONSTRAIN(iY,-iLimit,iLimit);
        dY = (errY - errYprv)/0.0001;
        yPID = 0.01*(yKp*errY + yKi*iY - yKd*fdY);    //scale 0.01(scale for 1)*50(max PWM) = 0.5
        iYprv = iY;
        errYprv = errY;

        rPID*=pwmxPID;
        pPID*=pwmxPID;
        yPID*=pwmxPID;
        
        rPID=CONSTRAIN(rPID,-pwmxPID,pwmxPID);
        pPID=CONSTRAIN(pPID,-pwmxPID,pwmxPID);
        yPID=CONSTRAIN(yPID,-pwmxPID,pwmxPID);

        m1 = throt + yPID + rPID + pPID ;
        m2 = throt - yPID - rPID + pPID ;
        m3 = throt + yPID - rPID - pPID ;
        m4 = throt - yPID + rPID - pPID ;

        // m1 = throt + rPID + pPID ;
        // m2 = throt - rPID + pPID ;
        // m3 = throt - rPID - pPID ;
        // m4 = throt + rPID - pPID ;

        // m1 = throt + rPID;
        // m2 = throt - rPID;
        // m3 = throt - rPID;
        // m4 = throt + rPID;

        m1 = CONSTRAIN(m1,0,255);
        m2 = CONSTRAIN(m2,0,255);
        m3 = CONSTRAIN(m3,0,255);
        m4 = CONSTRAIN(m4,0,255);

        if(motrState)
        {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, m1);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, m2);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, m3);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, m4);

            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
        }
        else
        {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0);

            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);

            iP=0;
            iR=0;
            iY=0;
        }

}

void IRAM_ATTR timer_callback(void* arg) 
{
    taskfunc();
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
    
    initfunc();
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

    // esp_rom_gpio_pad_select_gpio(GPIO_NUM_11);
    // gpio_set_direction(GPIO_NUM_11, GPIO_MODE_OUTPUT);
    initfunc();
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
        // printf("Yaw: %f, Pitch: %f, Roll: %f, dt: %f\n", yaw, pitch, roll, dt);
        printf("%.2f,%.2f,%.2f\n", roll, pitch, yaw);
        // printf("%.2f,%.2f,%.2f,%.2f\n",pitch, pPID,gy,iP);
        // printf("%.2f,%.2f,%.2f,%.2f,%.2f\n",roll,rPID,gx,fdR,iR);
        //  printf("%.2f,%.2f\n",dP,fdP);
        //  printf("%.2f,%.2f\n",dR,fdR);a
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// Function declarations from wifi_webserver.cpp
extern "C" void wifi_webserver_task(void* pvParameters);

// Main application, called from a specific core
extern "C" void app_main(void) 
{
    init_i2c();
    init_ledc_timer();
    // Initialize 4 LEDC channels for 4 GPIO pins
    init_ledc_channel(LEDC_CHANNEL_0, 17); 
    init_ledc_channel(LEDC_CHANNEL_1, 18); 
    init_ledc_channel(LEDC_CHANNEL_2, 33); 
    init_ledc_channel(LEDC_CHANNEL_3, 34); 


    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0);

    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);

    // ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 50);

    xTaskCreatePinnedToCore(&mpu6050_task, "mpu6050_task", 1024 * 8, NULL, 5, NULL, 0);
    // xTaskCreatePinnedToCore(&mpu6050_task_direct, "mpu6050_task_direct", 1024 * 8, NULL, 5, NULL, 0);

    xTaskCreatePinnedToCore(print_task, "print_task", 4096, NULL, 5, NULL, 1);

    xTaskCreatePinnedToCore(wifi_webserver_task, "wifi_webserver_task", 4096, NULL, 5, NULL, 1);
}


