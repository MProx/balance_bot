#pragma once

// I2C address of IMU:
#define MPU6050_I2C_ADDR 0x69

// Pin configuration:
#define BTN_L_PIN 0
#define V_IN_PIN 2 // Power measured after viltage divider
#define IMU_IRQ_PIN 12
#define IMU_SCL_PIN 13
#define IMU_SDA_PIN 15
#define MOTOR_2_STEP_PIN 25
#define MOTOR_2_DIR_PIN 26
#define MOTOR_EN_PIN 27
#define MOTOR_1_STEP_PIN 32
#define MOTOR_1_DIR_PIN 33
#define BTN_R_PIN 35

// Drive configuration:
#define MICROSTEPPING 32
#define STEPS_PER_REV 200
#define WHEEL_RADIUS 45 // mm

// I2C speed:
#define IMU_I2C_CLOCK_FREQUENCY 400000

// PID loop frequency:
#define PITCH_PID_LOOP_PERIOD 4000     // microseconds for 250 Hz loop
#define POSITION_PID_LOOP_PERIOD 20000 // microseconds for 50hz loop

// Power management
#define BATT_VOLTS_DIV_FACTOR 4.030 // Voltage divider (R1 = 100kohm and R2 = 33 kohm)
