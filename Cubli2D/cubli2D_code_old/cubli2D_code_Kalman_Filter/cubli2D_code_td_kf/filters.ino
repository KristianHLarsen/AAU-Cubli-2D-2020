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

void kalman_filter_setup() 
{
   I_kf << 1, 0, 0,
           0, 1, 0,
           0, 0, 1;
   B_kf << -0.0002,
           -0.2335, 
            3.0946;
   A_kf << 1.0002, 0.0020, 0.0000,
           0.1663, 0.9993, 0.0000,
          -0.1663, 0.0007, 0.9999;

   R_kf << 0.0100, 0.0000, 0.0000,
           0.0000, 1.0000, 0.0000,
           0.0000, 0.0000, 1000.0;
          
   Q_kf << 0.000001, 0.0000, 0.0000,
           0.0000, 1.0000, 0.0000,
           0.0000, 0.0000, 50.0000;


  x_prior_kf.Fill(0);             
  x_post_kf.Fill(9999); 
  K_kf.Fill(0);
  P_prior_kf = I_kf;
  P_post_kf = I_kf;
  H_kf = I_kf;          

}

void kalman_filter_update()
{
  GetIMUData();
  z_kf << az*PI/180,
          (GyZ * 0.0174532925),
          (((float)(((float)analogRead(SPEED_PIN) - 512)) * (2048.0 / 1024.0)) * rpm2rad);  
          
  if (x_post_kf(0) == 9999) x_post_kf = z_kf;
  
  x_prior_kf = A_kf * x_post_kf + B_kf*u_kf;
  P_prior_kf = A_kf * P_post_kf * (~A_kf) + Q_kf;
  K_kf = P_prior_kf * (~H_kf) * ((H_kf * P_prior_kf * (~H_kf) + R_kf)).Inverse();
  
  x_post_kf = x_prior_kf + K_kf * z_kf - H_kf * x_prior_kf;
  P_post_kf = P_prior_kf - K_kf * H_kf * P_prior_kf;   
}
