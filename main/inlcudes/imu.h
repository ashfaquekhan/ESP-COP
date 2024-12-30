#pragma once
#include "esp_err.h"
#include "driver/i2c.h"
#include "MPU6050.h"
#include "MadgwickAHRS.h"
#include "Kalman.h"

class IMU {
public:
    void init();
    void getMotion6(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);
    float getRoll() { return roll; }
    float getPitch() { return pitch; }
    float getYaw() { return yaw; }
    void update(float dt);

private:
    MPU6050 mpu;
    Madgwick madgwick;
    Kalman kalmanX, kalmanY;
    float roll, pitch, yaw;
    float accel_sensitivity = 16384.0;
    float gyro_sensitivity = 131.0;
};