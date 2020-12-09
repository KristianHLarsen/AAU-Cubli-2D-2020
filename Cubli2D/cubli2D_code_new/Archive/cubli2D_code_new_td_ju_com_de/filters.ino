void filter_setup() {
  GetIMUData();
  kc1 = ( 2 * tau - (samp_period / 1000000.0)) / (2 * tau + (samp_period / 1000000.0)); //Complementary filter
  kc2 = (samp_period / 1000000.0) / (2 * tau + (samp_period / 1000000.0));              //Complementary filter
  acc_angle_1[0] = az;
  acc_angle_1[1] = 0;
  gyro_angle_1[0] = GyZ;
  gyro_angle_1[1] = GyZ;
  comp_angle_1[0] = az;
  comp_angle_1[1] = 0;
}


void complementary() {
  kc1 = ( 2 * tau - ((sam_slut - sam_start) / 1000000.0)) / (2 * tau + ((sam_slut - sam_start) / 1000000.0));
  kc2 = ((sam_slut - sam_start) / 1000000.0) / (2 * tau + ((sam_slut - sam_start) / 1000000.0));
  GetIMUData();
  // Set old measurement data
  acc_angle_1[1] = acc_angle_1[0];
  gyro_angle_1[1] = gyro_angle_1[0];
  gyro_angle_1[0] = GyZ;
  comp_angle_1[1] = comp_angle_1[0];
  acc_angle_1[0] = az;
  comp_angle_1[0] = kc1 * comp_angle_1[1] + kc2 * (acc_angle_1[0] + acc_angle_1[1] + tau * gyro_angle_1[0] + tau * gyro_angle_1[1]) - 3.0 * 0.0174532925;
}
