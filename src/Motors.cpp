#include <Motors.h>

void Motors::begin()
{
    engine_.init();
    stepper1_ = engine_.stepperConnectToPin(MOTOR_1_STEP_PIN);
    stepper2_ = engine_.stepperConnectToPin(MOTOR_2_STEP_PIN);
    if (!stepper1_ || !stepper2_)
    {
        Serial.println("Failed to configure stepper(s)!");
        while (1)
            ;
    }
    stepper1_->setDirectionPin(MOTOR_1_DIR_PIN);
    stepper2_->setDirectionPin(MOTOR_2_DIR_PIN);
    stepper1_->setAcceleration(1000000); // Set high for responsive PID
    stepper2_->setAcceleration(1000000); // Set high for responsive PID

    pinMode(MOTOR_EN_PIN, OUTPUT);
    disable();
}

void Motors::stop()
{
    stepper1_->stopMove();
    stepper2_->stopMove();
}

void Motors::set_speed(int32_t pulse_speed_hz)
{
    stepper1_->setSpeedInHz(abs(pulse_speed_hz));
    stepper2_->setSpeedInHz(abs(pulse_speed_hz));
    if (pulse_speed_hz > 0)
    {
        stepper1_->runBackward();
        stepper2_->runForward();
    }
    else if (pulse_speed_hz < 0)
    {
        stepper1_->runForward();
        stepper2_->runBackward();
    }
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
