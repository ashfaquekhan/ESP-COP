#pragma once
#include "driver/ledc.h"
#include "esp_err.h"

class MotorController {
public:
    void init();
    void setMotorSpeeds(int m1, int m2, int m3, int m4);
    void stopMotors();
    bool isEnabled() { return enabled; }
    void setEnabled(bool state) { enabled = state; }

private:
    void initLedcTimer();
    void initLedcChannel(ledc_channel_t channel, int gpio_num);
    bool enabled = false;
};