#include "IMU.h"

void IMU::begin()
{
    imu_.initialize();
    if (!imu_.testConnection())
    {
        Serial.println("MPU6050 connection failed");
        while (true)
            ;
    }

    imu_.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
    imu_.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

    // Impirically measured offset correction:
    imu_.setXAccelOffset(-2745);
    imu_.setYAccelOffset(1056);
    imu_.setZAccelOffset(930);
    imu_.setXGyroOffset(52);
    imu_.setYGyroOffset(20);
    imu_.setZGyroOffset(-6);

    // Digital low-pass filrer to remove some noise:
    imu_.setDLPFMode(4); // Cutoff: ~41 Hz
}

void IMU::update()
{
    uint32_t _now = micros();

    // initialize timers on first call to avoid invalid result
    if (prev_time_ == 0)
    {
        prev_time_ = _now;
        pitch_rad = 0.0;
        pitch_rad_acc = 0.0;
        return;
    }
    float dTime = float(_now - prev_time_) / 1E6;
    prev_time_ = _now;

    // Update raw readings
    imu_.getMotion6(&ax_, &ay_, &az_, &gx_, &gy_, &gz_);

    // Apply low-pass filter to required accelerometer readings:
    axs_ = axs_ * 0.8 + ax_ * 0.2;
    azs_ = azs_ * 0.8 + az_ * 0.2;

    // Compute angle from accelerometer:
    pitch_rad_acc = atan2(-axs_, azs_);

    // Compute anglular rate from gyro. Convert from bits to rad / sec,
    // since full scale range (16 bits) is +/- 2000 deg/sec, aka 34.91 rad / sec
    pitch_rad_per_sec_gyro = float(gy_) * 34.91 / 32767;

    // Join sensor readings using complementary filter:
    pitch_rad = alpha_ * (pitch_rad + pitch_rad_per_sec_gyro * dTime) + (1.0 - alpha_) * pitch_rad_acc;

    // Debug (SLOWS DOWN LOOP):
    // Print raw IMU values:
    // Serial.printf("%i\t%i\t%i\t%i\t%i\t%i\n", ax_, ay_, az_, gx_, gy_, gz_);
    // Print comouted angles in degrees:
    // Serial.printf("-90, 90, %0.2f, %0.2f\n", pitch_rad_acc * RAD_TO_DEG, pitch_rad * RAD_TO_DEG);
}