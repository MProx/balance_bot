#pragma once

#include <Arduino.h>
#include "MPU6050.h"
#include "Config.h"
#include <Filters/Butterworth.hpp>

class IMU
{
public:
    IMU() : imu_(MPU6050_I2C_ADDR) {}
    float pitch_rad, pitch_rad_acc, pitch_rad_per_sec_gyro, yaw_rad_per_sec_gyro; // main outputs
    void begin();
    void update();

private:
    MPU6050 imu_;
    TwoWire *wire_;
    uint8_t addr_;

    uint32_t prev_time_ = 0; // For keeping track of time between measurements

    // for raw readings
    int16_t ax_, ay_, az_, gx_, gy_, gz_; // Raw IMU readings
};
