#pragma once

#include <Arduino.h>

#include "Config.h"

#include "FastAccelStepper.h"

class Motors
{
public:
    Motors() : engine_() {}
    void begin();
    void stop();
    void set_speed(int32_t speed);
    void enable();
    void disable();

private:
    FastAccelStepperEngine engine_;
    FastAccelStepper *stepper1_;
    FastAccelStepper *stepper2_;

    bool enabled_;
};
