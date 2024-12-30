#pragma once
#include "imu.h"
#include "motor_control.h"

class FlightController {
public:
    FlightController(IMU& imu, MotorController& motors);
    void init();
    void update(float dt);
    void setThrottle(int value) { throttle = value; }
    void setPIDGains(float rP, float rI, float rD, float pP, float pI, float pD);

private:
    void updatePID(float dt);
    float calculatePID(float error, float &integral, float &prevError,
                      float kp, float ki, float kd, float dt);
    
    IMU& imu;
    MotorController& motors;
    int throttle;
    float rollSetpoint = 0, pitchSetpoint = 0, yawSetpoint = 0;
    float rollIntegral = 0, pitchIntegral = 0, yawIntegral = 0;
    float prevRollError = 0, prevPitchError = 0, prevYawError = 0;
    
    // PID gains
    float rKp = 0.03, rKi = 0.0002, rKd = 0.05;
    float pKp = 0.03, pKi = 0.0002, pKd = 0.05;
    float yKp = 0.12, yKi = 0.0005, yKd = 0.0;
};