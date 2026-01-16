#include <Arduino.h>

#include "PIDController.h"

float PIDController::calculate(float input)
{
    uint32_t _now = micros();
    float _dInput = (input - prev_input_);
    float _dTime = float(_now - prev_time_derivative_) / 1e6;
    prev_time_derivative_ = _now;
    return calculate(input, _dInput / _dTime);
}

float PIDController::calculate(float input, float input_rate)
{
    uint32_t _now = micros();

    // bootstrap on first call:
    if (prev_time_ == 0)
    {
        prev_time_ = _now;
        return 0.0f;
    }

    float _error = setpoint_ - input;
    // Eliminate setpoint from dError to avoid lurches when setpoint changes:
    // _dError = (error - prev_error)
    //         = (setpoint - input) - (setpoint - prev_input )
    //         = prev_input - input;
    float _dError_dt = -1 * input_rate;

    // ----------
    // Proportional:
    P_ = kP_ * _error;

    // Integral:
    float _dTime = float(_now - prev_time_) / 1E6;
    I_ += kI_ * _error * _dTime;
    if (I_max_ != 0)
        I_ = constrain(I_, -I_max_, I_max_);

    // Differential:
    D_ = kD_ * _dError_dt;
    // ----------

    // Debug (SLOWS DOWN LOOP):
    // Serial.printf("%f\t%f\t%f\n", P_, I_, D_);

    // Save key values for calculus on next iteration:
    prev_time_ = _now;

    return P_ + I_ + D_;
}

void PIDController::set_setpoint(float setpoint)
{
    setpoint_ = setpoint;
}

// Constrain I term to prevent integral wind-up
void PIDController::constrain_I_term(float max_i_term)
{
    I_max_ = max_i_term;
}

void PIDController::reset_I_term()
{
    I_ = 0;
}