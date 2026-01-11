#include <Motors.h>

void Motors::begin()
{
    stepper1_.setMaxSpeed(100000);
    stepper2_.setMaxSpeed(100000);

    // Invert one direction pin, so that both wheels spin the same way
    stepper1_.setPinsInverted(true, false, false);

    pinMode(MOTOR_EN_PIN, OUTPUT);

    stop();
    initialised_ = true;
}

void Motors::stop()
{
    stepper1_.setSpeed(0);
    stepper2_.setSpeed(0);
}

void Motors::set_speed(int32_t pulse_speed_hz, int32_t yaw_diff)
{
    stepper1_.setSpeed(pulse_speed_hz + yaw_diff);
    stepper2_.setSpeed(pulse_speed_hz - yaw_diff);
}

void Motors::run()
{
    if (!initialised_)
        return;
    stepper1_.runSpeed();
    stepper2_.runSpeed();
}

void Motors::enable()
{
    if (!enabled_)
        digitalWrite(MOTOR_EN_PIN, LOW); // LOW == Enabled
    enabled_ = true;
}

void Motors::disable()
{
    if (enabled_)
    {
        stop();
        digitalWrite(MOTOR_EN_PIN, HIGH); // HIGH == Disabled
    }
    enabled_ = false;
}
