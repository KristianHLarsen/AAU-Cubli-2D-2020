void GetIMUData() {
  int i = 1;
  z = 0;
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B); //Specify address for accelerometer
  Wire.endTransmission(true);
  Wire.requestFrom(MPU_addr, 4, true);
  if (Wire.available()) {
    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
  }
  if (!Wire.available()) Wire.end();
  AcX = (AcX / 16384.0) * 9.81;
  AcY = (AcY / 16384.0) * 9.81;
  if (AcX > 1870.0)  AcX = AcX - 4000.0;
  if (AcY > 1870.0)  AcY = AcY - 4000.0;
  if (abs(AcY) < 0.1 || abs(AcX) < 0.1)  GetIMUData();
  az = -(((atan2(-AcY, -AcX) + PI) * RAD_TO_DEG) - 45);
  while (i <= mean) {
    Wire.begin();
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x47);   //Specify address for gyroscope
    Wire.endTransmission(true);
    Wire.requestFrom(MPU_addr, 2, true); // request a total of 14 registers
    if (Wire.available())  GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    if (!Wire.available()) Wire.end();
    if (GyZ > 32767)  GyZ = GyZ - 65535;  //Used to wrap values from max around 0
    GyZ = GyZ / scale;
    if (GyZ > 500) GyZ = 500;
    z += GyZ;
    i++;
  }
  GyZ = (z / mean);

}

float angle_pot() {
  if (sensor == 1 || sensor == 0) angle = ((float)(analogRead(potPin) - 4.0) * (90.0 / 1023.0)) - 45.0;
  if (sensor == 2) {
    complementary();
    angle = (comp_angle_1[0] + ANGLE_REF);
  } 
  if (angle > 45.0)angle = 45.0;    //move to +- 45 degrees
  if ( angle < -45.0)angle = -45.0;
  return (angle * 0.0174532925);
}
