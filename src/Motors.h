#pragma once

#include <Arduino.h>

#include "Config.h"

#include "AccelStepper.h"

class Motors
{
public:
    Motors() : stepper1_(AccelStepper::DRIVER, MOTOR_1_STEP_PIN, MOTOR_1_DIR_PIN),
               stepper2_(AccelStepper::DRIVER, MOTOR_2_STEP_PIN, MOTOR_2_DIR_PIN) {};
    void begin();
    void stop();
    void set_speed(int32_t pulse_speed_hz, int32_t yaw_diff);
    void enable();
    void disable();
    void run();

private:
    AccelStepper stepper1_;
    AccelStepper stepper2_;

    bool enabled_;
    bool initialised_ = false;
};
