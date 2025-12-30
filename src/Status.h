#pragma once

#include <Arduino.h>

typedef struct
{
    // horizontal speed PID coefficients:
    float speed_kP = 0.08537;
    float speed_kI = 0.08697;
    float speed_kD = 0.00541;

    // pitch angle PID coefficients:
    float pitch_kP = 37000;
    float pitch_kI = 1843000;
    float pitch_kD = 0;

    // Control variables:
    float v_in_volts;             // Batery voltage
    float pitch_rad_setpoint = 0; // Pitch angle setpoint
    float pitch_rad;              // IMU angle after complementary filter
    float pitch_rad_acc;          // IMU angle from accelerometer alone
    float pitch_rad_per_sec_gyro; // IMU anglular rate from gyro alone
    float speed = 0;              // Instantaneous linear speed
    float speed_setpoint = 0;     // Linear speed setpoint
    int8_t turning;               // Turning: -1 == left, +1 = right, 0 = straight
    bool loop_freq_warning;       // Set high when loop can't run in the desired frequency
} status_t;