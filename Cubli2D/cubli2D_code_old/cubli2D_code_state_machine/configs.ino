void IMUConfig() {
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); Wire.write(0);
  Wire.endTransmission(true);
  //Gyro config
  Wire.beginTransmission(MPU_addr);       //Contact IMU for setup
  Wire.write(0x1B);                       //GYRO_CONFIG register
  Wire.write(0x03);                       //Register bits set to b'00010000 (1000dps full scale)
  Wire.endTransmission(true);             //End transmission for gyro
  //Acc config
  Wire.beginTransmission(MPU_addr);       //Contact IMU for setup
  Wire.write(0x1C);                       //ACCEL_CONFIG register
  Wire.write(0b00000000);                 //full scale of the IMU
  Wire.endTransmission(true);
}

void PWMConfig() {
  FPGA.begin();
  FPGA.analogWriteResolution(bits, 4700);  //Roughly 5kHz signal
  FPGA.pinMode(PWM_PIN, 3);
  FPGA.analogWrite(PWM_PIN, map(freq_mid, 0, 100, pow(2, bits), 0)); //Set speed of wheel to 0 RPM
}
