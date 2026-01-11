#pragma once

#include <Arduino.h>

class PIDController
{
public:
    PIDController(float &kP, float &kI, float &kD) : kP_(kP), kI_(kI), kD_(kD) {};

    float calculate(float input, float input_rate);
    void set_setpoint(float setpoint);
    void constrain_I_term(float max_i_term);
    void reset_I_term();

private:
    float &kP_;
    float &kI_;
    float &kD_;
    float setpoint_ = 0;
    float P_, I_, D_;

    // Initialise vals which require memory between successive invokations
    float I_max_ = 0;
    float last_input_ = 0;
    uint32_t last_micros_ = 0;
};
