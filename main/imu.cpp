#include "imu.h"

void IMU::init() {
    mpu.initialize();
    mpu.setXAccelOffset(-707);
    mpu.setYAccelOffset(696);
    mpu.setZAccelOffset(1106);
    mpu.setXGyroOffset(118);
    mpu.setYGyroOffset(-31);
    mpu.setZGyroOffset(62);
    
    mpu.setRate(0);
    mpu.setExternalFrameSync(0);
    mpu.setDLPFMode(6);
    mpu.setFullScaleAccelRange(0);
    mpu.setFullScaleGyroRange(0);
}

void IMU::getMotion6(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    
    mpu.getMotion6(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);
    
    *ax = (float)accel_x / accel_sensitivity;
    *ay = (float)accel_y / accel_sensitivity;
    *az = (float)accel_z / accel_sensitivity;
    *gx = (float)gyro_x / gyro_sensitivity;
    *gy = (float)gyro_y / gyro_sensitivity;
    *gz = (float)gyro_z / gyro_sensitivity;
}

void IMU::update(float dt) {
    float ax, ay, az, gx, gy, gz;
    getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    madgwick.updateIMU(gx, gy, gz, ax, ay, az, dt);
    
    roll = madgwick.getRoll();
    pitch = madgwick.getPitch();
    yaw = gz;
}