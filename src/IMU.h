#pragma once

#include <Arduino.h>
#include "MPU6050.h"
#include "Config.h"

class IMU
{
public:
    IMU() : imu_(MPU6050_I2C_ADDR) {}
    float pitch_rad, pitch_rad_acc, pitch_rad_per_sec_gyro; // main outputs
    void begin();
    void update();

private:
    MPU6050 imu_;
    TwoWire *wire_;
    uint8_t addr_;

    uint32_t prev_time_ = 0; // For keepig track of time between measurements

    // for raw readings
    int16_t ax_, ay_, az_, gx_, gy_, gz_; // Raw IMU readings
    int16_t axs_, ays_, azs_;             // After low-pass filter

    uint8_t FIFOBuffer[64]; // FIFO storage buffer
    Quaternion q;           // [w, x, y, z] - Quaternion container
    float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector
    VectorFloat gravity;    // [x, y, z]            Gravity vector

    // const float alpha_ = 0.999; //  for complementary filter
    const float alpha_ = 0.98; //  for complementary filter
};
