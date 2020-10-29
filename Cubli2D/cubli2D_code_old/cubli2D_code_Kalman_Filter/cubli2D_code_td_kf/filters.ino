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

  Q_kf << 0.000001, 0.0000, 0.0000,
          0.0000, 1.0000, 0.0000,
          0.0000, 0.0000, 1.0000;
  R_kf << 0.0100, 0.0000, 0.0000,
          0.0000, 1.0000, 0.0000,
          0.0000, 0.0000, 1000.0;

  x_prior_kf.Fill(0);                
  x_post_kf.Fill(0); 
  K_kf.Fill(0);
  P_prior_kf = I_kf;
  P_post_kf = I_kf;
  H_kf = I_kf;            

}

void kalman_filter_update()
{
    x_prior(:,i) = A_d*x_post(:,i-1) + B_d*u(i);
    P_prior = A_d*P_post*transpose(A_d) + Q;
    K = P_prior * transpose(H) * ...
           inv((H * P_prior * transpose(H) + R));    
    x_post(:,i) = x_prior(:,i) + K*(z(:,i) - H*x_prior(:,i));
    P_post = P_prior - K*H*P_prior;

  x_prior_kf = A_kf * x_post_kf + B_kf*u_kf;
    
}
