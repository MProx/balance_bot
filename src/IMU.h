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

    uint32_t prev_time_ = 0; // For keepig track of time between measurements

    // for raw readings
    int16_t ax_, ay_, az_, gx_, gy_, gz_; // Raw IMU readings

    // buterworth filters:
    const float f_normalized = 2 * IMU_FILTER_CUTOFF_FREQ / (float(1e6) / PITCH_PID_LOOP_PERIOD);
    static const uint8_t sections = IMU_FILTER_ORDER == 2 ? 1U : IMU_FILTER_ORDER == 3 | IMU_FILTER_ORDER == 4 ? 2U
                                                                                                               : 3U;
    using filter_t = SOSFilter<float, sections, BiQuadFilterDF1<float>>;
    filter_t pitch_filter_ = butter<IMU_FILTER_ORDER>(f_normalized);
    filter_t pitch_rate_filter_ = butter<IMU_FILTER_ORDER>(f_normalized);
    filter_t yaw_rate_filter_ = butter<IMU_FILTER_ORDER>(f_normalized);
};
