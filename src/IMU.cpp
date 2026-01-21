#include "IMU.h"

void IMU::begin()
{
    pinMode(IMU_IRQ_PIN, INPUT);

    imu_.initialize();
    if (!imu_.testConnection())
    {
        Serial.println("MPU6050 connection failed");
        ESP.restart();
        while (true)
            ;
    }

    imu_.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    imu_.setFullScaleGyroRange(MPU6050_GYRO_FS_500);

    // Impirically measured offset correction:
    imu_.setXAccelOffset(1416);
    imu_.setYAccelOffset(-652);
    imu_.setZAccelOffset(1086);
    imu_.setXGyroOffset(21);
    imu_.setYGyroOffset(-1);
    imu_.setZGyroOffset(28);

    // Digital low-pass filrer to remove some noise:
    // imu_.setDLPFMode(4); // Cutoff: ~20 Hz

    imu_.setIntEnabled(1 << MPU6050_INTERRUPT_DATA_RDY_BIT);
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

    // Compute angle from accelerometer in degrees:
    pitch_rad_acc = atan2(-ax_, az_) * RAD_TO_DEG;

    // Compute anglular rates in deg / sec from gyro, since full scale range
    // (16 bits = 32767) codes to +/- 500 deg/sec
    pitch_rad_per_sec_gyro = float(gy_) * 500 / 32767;
    yaw_rad_per_sec_gyro = float(gz_) * 500 / 32767;

    // Join sensor readings using complementary filter:
    pitch_rad = (IMU_COMP_FILTER_ALPHA * (pitch_rad + pitch_rad_per_sec_gyro * dTime) +
                 (1.0 - IMU_COMP_FILTER_ALPHA) * pitch_rad_acc);

    // Debug (SLOWS DOWN LOOP):
    // Print raw IMU values:
    // Serial.printf("%i\t%i\t%i\t%i\t%i\t%i\n", ax_, ay_, az_, gx_, gy_, gz_);
    // Print comouted angles in degrees:
    // Serial.printf("-90, 90, %0.2f, %0.2f\n", pitch_rad_acc * RAD_TO_DEG, pitch_rad * RAD_TO_DEG);
}