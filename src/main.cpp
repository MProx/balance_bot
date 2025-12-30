#include <Arduino.h>
#include <Wire.h>

#include "Status.h"
#include "Config.h"
#include "PIDController.h"
#include "IMU.h"
#include "Motors.h"
#include "Display.h"
#include "Bluetooth.h"

status_t status;

PIDController speed_pid(status.speed_kP, status.speed_kI, status.speed_kD);
PIDController pitch_pid(status.pitch_kP, status.pitch_kI, status.pitch_kD);
IMU imu;
Motors motors;
Display display(TFT_WIDTH, TFT_HEIGHT, 1, &status);
Bluetooth bluetooth;

TaskHandle_t Task1; // For running task in core 0

void bt_message_handler(const uint8_t *, uint16_t);
void core0code(void *parameter);
float hz_to_speed(int16_t stepper_hz);

void setup()
{
  Serial.begin(115200);

  // Configure I2C
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN, IMU_I2C_CLOCK_FREQUENCY);

  // Run dedictaed core0code function on core 0
  xTaskCreatePinnedToCore(
      core0code, // Function object to implement the task
      "Task1",   // Name of the task
      10000,     // Stack size in words
      NULL,      // Task input parameter
      0,         // Priority of the task
      &Task1,    // Task handle.
      0);        // Core where the task should run

  // Configure peripheral devices:
  imu.begin();
  motors.begin();

  // Prevent integral windup. I term should only be needed for removing offset
  // that P term cannot take care of, so need not be big.
  speed_pid.constrain_I_term(10.0 * DEG_TO_RAD);
  pitch_pid.constrain_I_term(5500);
}

void loop()
{
  uint32_t loop_start_time = micros();

  // Bootstrap on first run:
  static uint32_t last_loop_start_time_ = 0;
  if (last_loop_start_time_ == 0)
  {
    last_loop_start_time_ = loop_start_time;
    return;
  }

  // Measure current pitch angle:
  imu.update();
  status.pitch_rad = imu.pitch_rad;                           // For P and I terms of pitch PID
  status.pitch_rad_per_sec_gyro = imu.pitch_rad_per_sec_gyro; // For D term of pitch PID
  status.pitch_rad_acc = imu.pitch_rad_acc;                   // For debugging, mainly

  // Avoid lurch at startup and only move if in active range
  const float limit = 45.0f * DEG_TO_RAD;
  if ((micros() < 1E6) || (abs(imu.pitch_rad) > limit))
  {
    motors.disable();
    status.speed = 0.0;
    status.speed_setpoint = 0.0;
    status.pitch_rad_setpoint = 0.0;
  }
  else
  {
    // TODO: Determine speed setpoint from BLE commands for remote control
    // TODO: figure out turning.
    status.speed_setpoint = 0.0; // mm/s
    status.turning = 0;

    // Calculate requried pitch angle to achieve desired linear speed
    speed_pid.set_setpoint(status.speed_setpoint);
    status.pitch_rad_setpoint = speed_pid.calculate(status.pitch_rad);

    // Calculate requried stepper pulse speed to achieve desired pitch angle
    pitch_pid.set_setpoint(status.pitch_rad_setpoint);
    float output = pitch_pid.calculate(status.pitch_rad);

    // Run motors at calculated speed.
    // Note, motors are inverting outout (positive motor speed tends to tip the
    // robot in the negative direction). So use negative output.
    motors.enable();
    motors.set_speed(-output);
    status.speed = hz_to_speed(-output);
  }

  last_loop_start_time_ = loop_start_time;

  // Try to maintain a consistent loop frequency:
  bool warning = true;
  while (micros() - loop_start_time < PID_LOOP_PERIOD)
    warning = false;
  if (warning)
    status.loop_freq_warning = true;
}

// Function to run on Core0
void core0code(void *parameter)
{
  display.begin();
  bluetooth.begin(bt_message_handler);

  while (true)
  {
    status.v_in_volts = analogReadMilliVolts(V_IN_PIN) * V_IN_VOLTAGE_DIV_FACTOR / 1000.0;
    bluetooth.update();
    display.update();
    delay(1); // Feed watchdog
  }
}

// Callback function invoked when bluetooth message is recieved:
void bt_message_handler(const uint8_t *msg, uint16_t len_msg)
{
  // Check message format. Expect "[key char] [value]\n", e.g. "P 0.12345\n"
  if (len_msg < 4 || msg[1] != ' ' || msg[len_msg - 1] != '\n')
  {
    Serial.println("WARNING: Bad message format");
    return;
  }

  // Extract key and value of message:
  char msg_key = msg[0];
  uint16_t value_len = len_msg - 3; // exclude key char, ' ', and '\n'
  char value_buf[value_len];
  memcpy(value_buf, &msg[2], value_len);
  value_buf[value_len] = '\0';
  float msg_value = atof(value_buf);

  // Handle key and value:
  switch (msg_key)
  {
  case 'P':
    status.pitch_kP = msg_value;
    Serial.printf("Pitch P set to %f\n", msg_value);
    break;

  case 'I':
    status.pitch_kI = msg_value;
    Serial.printf("Pitch I set to %f\n", msg_value);
    break;

  case 'D':
    status.pitch_kD = msg_value;
    Serial.printf("Pitch D set to %f\n", msg_value);
    break;

  case 'p':
    status.speed_kP = msg_value;
    Serial.printf("Speed P set to %f\n", msg_value);
    break;

  case 'i':
    status.speed_kI = msg_value;
    Serial.printf("Speed I set to %f\n", msg_value);
    break;

  case 'd':
    status.speed_kD = msg_value;
    Serial.printf("Speed D set to %f\n", msg_value);
    break;
  default:
    Serial.println("Unkloop_start_timen message key");
  }
}

// Helper function to determine horizontal speed from stepper pulse frequency
float hz_to_speed(int16_t stepper_hz)
{
  // Convert driver step pulses to wheel radians per second:
  float wheel_speed = (float)stepper_hz / MICROSTEPPING / STEPS_PER_REV * TWO_PI;

  // Convert wheel angular speed to robot linear speed:
  float linear_speed = wheel_speed * WHEEL_RADIUS;

  return linear_speed; // mm/s
}