#include <Arduino.h>

#include "PIDController.h"

float PIDController::calculate(float input)
{
    uint32_t _now = micros();

    // initialize timers/input on first call to avoid huge/invalid dTerm
    if (last_micros_ == 0)
    {
        last_micros_ = _now;
        last_input_ = input;
        return 0.0f;
    }

    float _dTime = float(_now - last_micros_) / 1E6;

    float _error = setpoint_ - input;
    // Eliminate setpoint from dError to avoid lurches when setpoint changes:
    // _dError = (_error - last_error)
    //         = (setpoint_ - input) - (setpoint_ - last_input_ )
    float _dError = last_input_ - input;

    // Save key values for calculus on next iteration:
    last_micros_ = _now;
    last_input_ = input;

    // ----------
    // Proportional:
    P_ = kP_ * _error;

    // Integral:
    I_ += kI_ * _error * _dTime;
    if (I_max_ != 0)
        I_ = constrain(I_, -I_max_, I_max_);

    // Differential:
    D_ = kD_ * _dError / _dTime;

    // ----------

    // Debug (SLOWS DOWN LOOP):
    // Serial.printf("%f\t%f\t%f\n", P_, I_, D_);

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