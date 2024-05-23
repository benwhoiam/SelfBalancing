#include <Wire.h>
#include <SoftwareSerial.h>
#include "timer_handler.h"
#include "buzzer_handler.h"

#define DEBUG false
#if !DEBUG
#define log(...)
#else
#define log(fmt, ...) consoleLog(fmt, ##__VA_ARGS__)
#endif

// CONSTS
#define ACC_CALIBRATION_VALUE -8450 // 200
// PID
float Kp = 7;   // 10
float Ki = 0.5; // was 1.5
float Kd = 38;  // 30
// Speed
#define turning_speed 30
#define max_target_speed 150

// PINS
// Only use port 0 to 7;
#define dirLeft 3
#define stepLeft 2
#define dirRight 5
#define stepRight 4

#define TXpin 7
#define RXpin 8
#define STATEpin 12

#define BUZZER_PIN 11
//------------------------------------------------
#define dirRightBackward 0b00000000 | 1 << dirRight     // 00100000
#define dirRightForward (0b11111111 & ~(1 << dirRight)) // 11011111
#define stepRightHIGH 0b00000000 | 1 << stepRight       // 00010000
#define stepRightLOW (0b11111111 & ~(1 << stepRight))   // 11101111
#define dirLeftForward 0b00000000 | 1 << dirLeft        // 00001000
#define dirLeftBackward (0b11111111 & ~(1 << dirLeft))  // 11110111
#define stepLeftHIGH 0b00000000 | 1 << stepLeft         // 00000100
#define stepLeftLOW (0b11111111 & ~(1 << stepLeft))     // 11111011
#define GYRO_ADDRESS 0x68
//------------------------------------------------
SoftwareSerial HC05(TXpin, RXpin); // TX | RX of hc05
//------------------------------------------------

byte start;
char received_byte;

int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;

int receive_counter;
int gyro_pitch_data_raw, gyro_yaw_data_raw, accelerometer_data_raw;

long gyro_yaw_calibration_value, gyro_pitch_calibration_value;

unsigned long loop_timer;

float angle_gyro, angle_acc, angle, self_balance_pid_setpoint;
float pid_error, pid_i_mem, pid_setpoint, gyro_input, pid_output, last_pid_error;
float pid_output_left, pid_output_right;

bool want_to_use_state = true;
bool hc05_is_connected = false;
bool last_hc05_is_connected = !hc05_is_connected;

void recieveData()
{
  if (HC05.available())
  {
    received_byte = HC05.read();
    log("recieved: ", received_byte);
    if (received_byte == 'p')
    {
      Kp += 1;
      return;
    }
    else if (received_byte == ';')
    {
      Kp -= 1;
      return;
    }
    else if (received_byte == 'i')
    {
      Ki += 0.1;
      return;
    }
    else if (received_byte == 'k')
    {
      Ki -= 0.1;
      return;
    }
    else if (received_byte == 'y')
    {
      Kd += 1;
      return;
    }
    else if (received_byte == 'h')
    {
      Kd -= 1;
      return;
    }
    else if (received_byte == 't')
    {
      want_to_use_state = !want_to_use_state;
      // buzzer::beep(1, 70);
      return;
    }
    else if (received_byte == 'b')
    {
      digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));
      log("buzzer: ", digitalRead(BUZZER_PIN));
      return;
    }
    log("received_byte", received_byte);
    receive_counter = 0;
  }
  int tilt_time = 15;
  if (received_byte == "w" || received_byte == "s")
  {
    tilt_time = 150;
  }
  else if (received_byte == "a" || received_byte == "d")
  {
    tilt_time = 15;
  }
  if (receive_counter <= tilt_time)
    receive_counter++; // The received byte will be valid for 25 program loops (100ms)
  else
    received_byte = '\0'; // After 100 ms the received byte is deleted
}

void handleStart()
{
  if (start == 0 && angle_acc > -0.5 && angle_acc < 0.5)
  {
    angle_gyro = angle_acc;
    start = 1;
  }
}

void calculatePID()
{
  pid_error = angle_gyro - self_balance_pid_setpoint - pid_setpoint;
  if (pid_output > 10 || pid_output < -10)
    pid_error += pid_output * 0.015; // this will resiste change when pushed by hand

  pid_i_mem /* = Ki*pid_error + Ki*last_pid_error; */ /*was*/ += Ki * pid_error;
  pid_i_mem = pid_i_mem > 400 ? 400 : ((pid_i_mem < -400) ? -400 : pid_i_mem);

  pid_output = Kp * pid_error + pid_i_mem + Kd * (pid_error - last_pid_error);
  pid_output = pid_output > 400 ? 400 : ((pid_output < -400) ? -400 : pid_output);

  last_pid_error = pid_error;
}
void handleTipOver(int maxTilte = 30)
{
  // If the robot tips over or the start variable is zero
  if (angle_gyro > maxTilte || angle_gyro < -maxTilte)
  {
    pid_output = 0;
    pid_i_mem = 0;
    self_balance_pid_setpoint = 0;
    if (start == 0)
      return;
    start = 0;
    buzzer::beep(5, 40);
  }
}
void handleNotStart()
{
  if (start == 0)
  {
    pid_output = 0;
    pid_i_mem = 0;
    start = 0;
    self_balance_pid_setpoint = 0;
  }
}
void handleVirbation(int err = 5)
{
  // When very close to 5, no need to vibrate.
  if (pid_output < err && pid_output > -err)
    pid_output = 0;
}

void calculateControls(float setTilte = 2.5)
{
  pid_output_left = pid_output;
  pid_output_right = pid_output;

  if (received_byte == 'a')
  {
    pid_output_left += turning_speed;
    pid_output_right -= turning_speed;
  }
  else if (received_byte == 'd')
  {
    pid_output_left -= turning_speed;
    pid_output_right += turning_speed;
  }
  else if (received_byte == 'w')
  {
    if (pid_setpoint > -setTilte)
      pid_setpoint -= 0.2; // 0.05
    if (pid_output > max_target_speed * -1)
      pid_setpoint -= 0.005;
  }
  else if (received_byte == 's')
  {
    if (pid_setpoint < setTilte)
      pid_setpoint += 0.2; // 0.05
    if (pid_output < max_target_speed)
      pid_setpoint += 0.005; // 0.005
  }

  if (received_byte != 'w' && received_byte != 's')
  {
    if (pid_setpoint > 0.5)
      pid_setpoint -= 0.05; // was 0.05
    else if (pid_setpoint < -0.5)
      pid_setpoint += 0.05; // was 0.05
    else
      pid_setpoint = 0;
  }
}
void handleTiltedSurfaces()
{
  // The self balancing point is adjusted when there is not forward or backwards movement from the transmitter. This way the robot will always find it's balancing point
  if (pid_setpoint == 0)
  { // If the setpoint is zero degrees
    if (pid_output < 0)
      self_balance_pid_setpoint += 0.004; // 0.015 is good   //was0.0015              //Increase the self_balance_pid_setpoint if the robot is still moving forewards
    if (pid_output > 0)
      self_balance_pid_setpoint -= 0.004; // Decrease the self_balance_pid_setpoint if the robot is still moving backwards
  }
}

int convertToLinear(float to_convert)
{
  to_convert = (to_convert > 0) ? 405 - (1 / (to_convert + 9)) * 5500 : ((to_convert < 0) ? -405 - (1 / (to_convert - 9)) * 5500 : to_convert);
  // Calculate the needed pulse time for the left and right stepper motor controllers
  return (to_convert > 0) ? 400 - to_convert : ((to_convert < 0) ? -400 - to_convert : 0);
}
void maintainLoopDuration()
{
  while (loop_timer > micros())
    ;
  loop_timer += 4000;
}
void handleHC05Connetion()
{
  hc05_is_connected = digitalRead(STATEpin);
  if (hc05_is_connected != last_hc05_is_connected)
  {
    last_hc05_is_connected = hc05_is_connected;
    if (hc05_is_connected)
    {
      // buzzer::beep(2, 50);
    }
    else if (!hc05_is_connected)
    {
      // buzzer::beep(4, 500);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  HC05.begin(9600);
  log("~ Setup Started ~");
  Wire.begin();
  timer::setup();
  buzzer::setup(BUZZER_PIN);
  initGyro();
  pinMode(stepLeft, OUTPUT);
  pinMode(dirLeft, OUTPUT);
  pinMode(stepRight, OUTPUT);
  pinMode(dirRight, OUTPUT);
  pinMode(13, OUTPUT);
  initGyroCalibration(500);
  log("~ Setup Done ~");
  buzzer::beep(2, 40);
  loop_timer = micros() + 4000; // loop shall always take 4ms <-> 250Hz
}

void loop()
{
  buzzer::HandleBuzzer();
  // handleHC05Connetion();
  recieveData();
  calculateAccelerometerAngle();
  handleStart();
  calculateGyroAngle();
  calculatePID();
  handleTipOver();
  handleNotStart();
  handleVirbation();
  calculateControls();

  handleTiltedSurfaces();
  throttle_left_motor = convertToLinear(pid_output_left);
  throttle_right_motor = convertToLinear(pid_output_right);
  maintainLoopDuration();
  // log("angle_gyro:", angle_gyro);
}
