#include "motor_control.h"

void MotorController::init() {
    initLedcTimer();
    initLedcChannel(LEDC_CHANNEL_0, 17);
    initLedcChannel(LEDC_CHANNEL_1, 18);
    initLedcChannel(LEDC_CHANNEL_2, 33);
    initLedcChannel(LEDC_CHANNEL_3, 34);
    stopMotors();
}

void MotorController::initLedcTimer() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_2,
        .freq_hz = 7000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);
}

void MotorController::initLedcChannel(ledc_channel_t channel, int gpio_num) {
    ledc_channel_config_t ledc_channel = {
        .gpio_num = gpio_num,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_2,
        .duty = 0,
        .hpoint = 0,
        .flags = {.output_invert = 0}
    };
    ledc_channel_config(&ledc_channel);
}

void MotorController::setMotorSpeeds(int m1, int m2, int m3, int m4) {
    if (!enabled) {
        stopMotors();
        return;
    }

    // Constrain values between 0 and 250
    m1 = m1 < 0 ? 0 : (m1 > 250 ? 250 : m1);
    m2 = m2 < 0 ? 0 : (m2 > 250 ? 250 : m2);
    m3 = m3 < 0 ? 0 : (m3 > 250 ? 250 : m3);
    m4 = m4 < 0 ? 0 : (m4 > 250 ? 250 : m4);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, m1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, m2);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, m3);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, m4);

    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
}

void MotorController::stopMotors() {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, 0);

    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
}
