#include <Arduino.h>
#include <Wire.h>

#include "Config.h"
#include "Status.h"
#include "PIDController.h"
#include "IMU.h"
#include "Motors.h"
#include "Display.h"
#include "Bluetooth.h"

status_t status;

IMU imu;
Motors motors;
Display display(TFT_WIDTH, TFT_HEIGHT, 1, &status);
Bluetooth bluetooth;
PIDController position_pid(status.position_kP, status.position_kI, status.position_kD);
PIDController pitch_pid(status.pitch_kP, status.pitch_kI, status.pitch_kD);
PIDController yaw_pid(status.yaw_kP, status.yaw_kI, status.yaw_kD);

TaskHandle_t Task1; // For running task in core 0

void bt_message_handler(const uint8_t *, uint16_t);
void core0code(void *parameter);
float hz_to_speed(int16_t stepper_hz);
float get_batt_volts();

// buterworth filters:
const float f_normalized_imu = 2 * IMU_FILTER_CUTOFF_FREQ / (float(1e6) / PITCH_PID_LOOP_PERIOD);
auto pitch_filter_ = butter<IMU_FILTER_ORDER>(f_normalized_imu);
auto pitch_rate_filter_ = butter<IMU_FILTER_ORDER>(f_normalized_imu);
auto yaw_rate_filter_ = butter<IMU_FILTER_ORDER>(f_normalized_imu);

void setup()
{
  Serial.begin(115200);

  pinMode(BTN_L_PIN, INPUT_PULLUP);
  pinMode(BTN_R_PIN, INPUT_PULLUP);

  // Configure peripheral devices:
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN, IMU_I2C_CLOCK_FREQUENCY);
  imu.begin();
  attachInterrupt(digitalPinToInterrupt(IMU_IRQ_PIN), []()
                  { status.imu_data_ready = true; }, RISING);
  motors.begin();
  motors.disable();

  // Run dedictaed core0code function on core 0
  xTaskCreatePinnedToCore(
      core0code, // Function object to implement the task
      "Task1",   // Name of the task
      10000,     // Stack size in words
      NULL,      // Task input parameter
      0,         // Priority of the task
      &Task1,    // Task handle.
      0);        // Core where the task should run

  // Prevent integral windup. I term should only be needed for removing offset
  // that P term cannot take care of, so need not be big.
  position_pid.constrain_I_term(4.0);
  pitch_pid.constrain_I_term(20000);
}

void loop()
{
  uint32_t now = micros();

  // Bootstrap from previous loops:
  static uint32_t prev_pitch_pid_time = now;
  static uint32_t prev_pos_pid_time = now;

  // Run pitch PID loop rapidly:
  if (now - prev_pitch_pid_time > PITCH_PID_LOOP_PERIOD)
  {
    // Retrieve IMU readings:
    if (status.imu_data_ready)
    {
      imu.update();
      status.pitch_rad = pitch_filter_(imu.pitch_rad);
      status.pitch_rate_rad_per_sec = pitch_rate_filter_(imu.pitch_rad_per_sec_gyro);
      status.yaw_rate_rad_per_sec = yaw_rate_filter_(imu.yaw_rad_per_sec_gyro);
      status.imu_data_ready = false;
    }

    // Avoid lurch at startup and only move if in active range
    if ((micros() < STARTUP_DELAY_US) || (abs(imu.pitch_rad) > PITCH_LIMIT || (!digitalRead(BTN_R_PIN))))
    {
      motors.disable();
      status.position = 0.0;
      status.position_setpoint = 0.0;
      status.pitch_rad_setpoint = 0.0;
      pitch_pid.reset_I_term();
      position_pid.reset_I_term();
    }
    else
    {
      // Calculate requried stepper pulse frequency to achieve desired pitch angle
      pitch_pid.set_setpoint(status.pitch_rad_setpoint);
      float pitch_output_raw = pitch_pid.calculate(status.pitch_rad, status.pitch_rate_rad_per_sec);
      status.pitch_output = status.pitch_output * 0.95 + pitch_output_raw * 0.05; // Crude low-pass filter

      // Run motors at calculated pulse frequency.
      // Note, motors are inverting outout (positive motor speed tends to tip the
      // robot in the negative direction). So use negative output.
      motors.enable();
      status.yaw_offset = 0;
      motors.set_speed(-status.pitch_output, status.yaw_offset);

      // Update position:
      float dT = float(now - prev_pitch_pid_time) / 1e6;
      status.speed = hz_to_speed(-status.pitch_output);
      status.position += status.speed * dT;
    }

    prev_pitch_pid_time = now;
  }

  if (now - prev_pos_pid_time > POSITION_PID_LOOP_PERIOD)
  {
    // Calculate requried pitch angle to achieve desired linear position
    // Note: position setpoint comes from BT commands for remote control
    position_pid.set_setpoint(status.position_setpoint);
    static float speed_s = 0;
    speed_s = status.speed * 0.999 + speed_s * 0.001; // crude low-pass filter
    status.pitch_rad_setpoint = position_pid.calculate(status.position, speed_s);

    // Calculate required motor differential pulse rate for yaw offset:
    // Note: yaw rate setpoint comes from BT commands for remote control
    yaw_pid.set_setpoint(status.yaw_rate_setpoint);
    status.yaw_offset = yaw_pid.calculate(status.yaw_rate_rad_per_sec);

    prev_pos_pid_time = now;
  }
}

// Function to run on Core0
void core0code(void *parameter)
{
  bluetooth.begin(bt_message_handler);
  while (true)
  {
    if (status.check_bt || !digitalRead(BTN_L_PIN))
    {
      bluetooth.update(); // Blocks with active BT connection!
      status.check_bt = true;
      // status.yaw_rate_setpoint = 20;
    }
    motors.run();
    delayMicroseconds(0); // feed watchdog
  }
}

// Callback function invoked when bluetooth message is received:
void bt_message_handler(const uint8_t *msg, uint16_t len_msg)
{
  // Check message format. Expect "[key char] [value float]\n", e.g. "P 12.345\n"
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
    status.position_kP = msg_value / 1000.0;
    Serial.printf("Pos P set to %f\n", msg_value / 1000.0);
    break;

  case 'i':
    status.position_kI = msg_value / 1000.0;
    Serial.printf("Pos I set to %f\n", msg_value / 1000.0);
    break;

  case 'd':
    status.position_kD = msg_value / 1000.0;
    Serial.printf("Pos D set to %f\n", msg_value / 1000.0);
    break;

    // case 'x':
    //   status.position = msg_value;
    //   break;

  case 'y':
    status.position_setpoint += msg_value;
    break;

  default:
    Serial.println("Unknown message key");
  }
}

// Helper function to determine horizontal speed from stepper pulse frequency
float hz_to_speed(int16_t stepper_hz)
{
  float wheel_speed = (float)stepper_hz / MICROSTEPPING / STEPS_PER_REV * TWO_PI;
  float linear_speed = wheel_speed * WHEEL_RADIUS;
  return linear_speed; // mm/s
}

float get_batt_volts()
{
  float batt_volts_raw = analogReadMilliVolts(V_IN_PIN) * BATT_VOLTS_DIV_FACTOR / 1000.0;

  // Apply smoothing:
  static float batt_volts_smooth = batt_volts_raw;
  const float alpha = 0.9;
  batt_volts_smooth = batt_volts_smooth * alpha + batt_volts_raw * (1 - alpha);

  return batt_volts_smooth;
}