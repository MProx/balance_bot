#pragma once

#include <Arduino.h>

typedef struct
{
    // pitch angle PID coefficients:
    float pitch_kP = 1715.0;
    float pitch_kI = 3887.0;
    float pitch_kD = 100.0;

    // horizontal position PID coefficients:
    float position_kP = 0.0020;
    float position_kI = 0.0;
    float position_kD = 0.0095;

    // yaw angle PID coefficients:
    float yaw_kP = 40.0;
    float yaw_kI = 5.0;
    float yaw_kD = 0.0;

    // Control variables:
    float batt_volts;               // Batery voltage
    float pitch_rad_setpoint = 0;   // Pitch angle setpoint
    float position_setpoint = 0;    // Linear position setpoint
    float yaw_rate_setpoint = 0;    // Yaw rate setpoint (deg/sec)
    float pitch_rad, pitch_rad_acc; // IMU angle after complementary filter
    float pitch_rate_rad_per_sec;   // IMU anglular rate from gyro alone
    float yaw_rate_rad_per_sec;     // IMU anglular rate from gyro alone
    float speed = 0;                // Instantaneous linear speed
    float position = 0;             // Instantaneous linear position
    float pitch_output;             // Motor pulse rate for pitch control
    float yaw_offset = 0.35;        // // Differential motor pulse rate for yaw = 20 deg / sec
    bool imu_data_ready = false;
    bool check_bt = false;
} status_t;