
void initGyro() {
  //By default the MPU-6050 sleeps. So we have to wake it up.
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(0x6B);  //scr8
  Wire.write(0x00);
  Wire.endTransmission();
  //Set the full scale of the gyro to +/- 250 degrees per second
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();
  //Set the full scale of the accelerometer to +/- 4g.
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(0x1C);
  Wire.write(0x08);
  Wire.endTransmission();
  //Set some filtering to improve the raw data.
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();
}

void initGyroCalibration(int reps) {
  for (receive_counter = 0; receive_counter < reps; receive_counter++) {
    if (receive_counter % 15 == 0) digitalWrite(13, !digitalRead(13));
    Wire.beginTransmission(GYRO_ADDRESS);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(GYRO_ADDRESS, 4);
    gyro_yaw_calibration_value += Wire.read() << 8 | Wire.read();
    gyro_pitch_calibration_value += Wire.read() << 8 | Wire.read();
    delayMicroseconds(3700);
  }
  gyro_pitch_calibration_value /= reps;
  gyro_yaw_calibration_value /= reps;
}

void calculateAccelerometerAngle() {
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(0x3F);
  Wire.endTransmission();

  Wire.requestFrom(GYRO_ADDRESS, 2);
  accelerometer_data_raw = Wire.read() << 8 | Wire.read();
  //Serial.print("gyro_acc:");
  //Serial.println(accelerometer_data_raw);
  accelerometer_data_raw += ACC_CALIBRATION_VALUE;
  // Limiting the acc data to +/-8200;
  accelerometer_data_raw = (accelerometer_data_raw > 8200) ? 8200 : ((accelerometer_data_raw < -8200) ? -8200 : accelerometer_data_raw);
  //Calculate the current angle according to the accelerometer
  angle_acc = asin((float)accelerometer_data_raw / 8200.0) * 57.296;  //57 = 180/pi
}

void calculateGyroAngle() {
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(GYRO_ADDRESS, 4);
  gyro_yaw_data_raw = Wire.read() << 8 | Wire.read();
  gyro_pitch_data_raw = Wire.read() << 8 | Wire.read();

  gyro_pitch_data_raw -= gyro_pitch_calibration_value;
  angle_gyro += gyro_pitch_data_raw * 0.000031;  //Calculate the traveled angle during this loop  and add this to the angle_gyro variable
  gyro_yaw_data_raw -= gyro_yaw_calibration_value;
  angle_gyro -= gyro_yaw_data_raw * 0.0000003;  //Compensate the gyro offset when the robot is rotating
  angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004;
}