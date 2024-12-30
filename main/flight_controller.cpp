#include "flight_controller.h"

FlightController::FlightController(IMU& imu, MotorController& motors) 
    : imu(imu), motors(motors), throttle(0) {}

void FlightController::init() {
    rollSetpoint = 0;
    pitchSetpoint = 0;
    yawSetpoint = 0;
    rollIntegral = pitchIntegral = yawIntegral = 0;
    prevRollError = prevPitchError = prevYawError = 0;
}

float FlightController::calculatePID(float error, float &integral, float &prevError,
                                   float kp, float ki, float kd, float dt) {
    integral += error * dt;
    integral = integral < -100000 ? -100000 : (integral > 100000 ? 100000 : integral);
    
    float derivative = (error - prevError) / dt;
    prevError = error;
    
    return kp * error + ki * integral + kd * derivative;
}

void FlightController::update(float dt) {
    imu.update(dt);
    
    float roll = imu.getRoll();
    float pitch = imu.getPitch();
    float yaw = imu.getYaw();
    
    // Calculate errors
    float rollError = rollSetpoint - roll;
    float pitchError = pitchSetpoint - pitch;
    float yawError = yawSetpoint - yaw;
    
    // Calculate PID outputs
    float rollOutput = calculatePID(rollError, rollIntegral, prevRollError, rKp, rKi, rKd, dt);
    float pitchOutput = calculatePID(pitchError, pitchIntegral, prevPitchError, pKp, pKi, pKd, dt);
    float yawOutput = calculatePID(yawError, yawIntegral, prevYawError, yKp, yKi, yKd, dt);
    
    // Constrain outputs
    rollOutput = rollOutput < -100 ? -100 : (rollOutput > 100 ? 100 : rollOutput);
    pitchOutput = pitchOutput < -100 ? -100 : (pitchOutput > 100 ? 100 : pitchOutput);
    yawOutput = yawOutput < -100 ? -100 : (yawOutput > 100 ? 100 : yawOutput);
    
    // Mix outputs to motor values
    int m1 = throttle + rollOutput - pitchOutput + yawOutput;
    int m2 = throttle - rollOutput - pitchOutput - yawOutput;
    int m3 = throttle - rollOutput + pitchOutput + yawOutput;
    int m4 = throttle + rollOutput + pitchOutput - yawOutput;
    
    motors.setMotorSpeeds(m1, m2, m3, m4);
}

void FlightController::setPIDGains(float rP, float rI, float rD, float pP, float pI, float pD) {
    rKp = rP; rKi = rI; rKd = rD;
    pKp = pP; pKi = pI; pKd = pD;
}