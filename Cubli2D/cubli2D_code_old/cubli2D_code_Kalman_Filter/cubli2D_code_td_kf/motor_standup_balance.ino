float interpolate(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// returns speed of frame based on sensor
float speed_frame() {
  if (sensor == 1 || sensor == 0) {
    time_now = micros();
    angle_speed = ((ang_err - angle_last) / ((float)(time_now - time_last) / 1000000.0)); //0.0018
    time_last = time_now;
    angle_last = ang_err;
  }
  if (sensor == 2) angle_speed = (GyZ * 0.0174532925);  //Gyroscope is speed of frame
  return angle_speed;
}

// returns gyro speed of frame
float speed_frame_imu() {
  return (GyZ * 0.0174532925); 
}

void standup() {
  sensor = 1;
  if (abs(angle_pot()) > recovery) {  // if we are down
    FPGA.analogWrite(PWM_PIN, map(50, 0, 100, pow(2, bits), 0)); //stop motor
    delay(200);        //Let it fall completely
    if (abs((((float)(((float)analogRead(SPEED_PIN) - 512)) * (2048.0 / 1024.0)) * rpm2rad)) > 100.0) {
      for (int o = 0; o < 11; o++) { // if we are above a certain speed use ABS brake for not tipping over
        brake.write(brk + 3);
        delay(30);
        brake.write(go);
        delay(50);
      }
    }
    brake.write(brk);
    delay(300);
    brake.write(go);
    if (angle_pot() > 0) { //determine the way we fell and spin up accordingly
      FPGA.analogWrite(PWM_PIN, map(45, 0, 100, pow(2, bits), 0)); //Determine the direction we fell
      delay(3000);
    }
    else {
      FPGA.analogWrite(PWM_PIN, map(55, 0, 100, pow(2, bits), 0));
      delay(3500);
    }
    brake.write(brk - 5); //brake motor hard to get up properly
    delay(75);            //wait before disabling the brake
    brake.write(go);      // release brake
    int g = micros();
    add_cycle = true;     //Disable ANGLE_REF corrections
    while (micros() - g < 1000000) {   //wait for system to be stable before giving the IMU control
      updateMotor();
      delay(10);
    }
    filter_setup();
    add_cycle = false;     //ENABLE ANGLE_REF corrections
    sam_start = micros();  //Reset timing parameter since last reading
    timer_var = micros();  // --//--
    if (abs(ANGLE_REF) > 15.0) { // if the angle ref somehow changed a lot, move it back.
      if (ANGLE_REF > 0.0) ANGLE_REF = 14.0;
      if (ANGLE_REF < 0.0) ANGLE_REF = -14.0;
    }
  }
  sensor = ogsens;         //Reset to original sensor before standup
}

void balancePoint() { // Change ANGLE_REF if we are at constant wheel velocity
  if (cycle == CHECK_CYCLE) { // if we have enough readings
    for (int x = 0; x < CHECK_CYCLE; x++)   running_average = running_average + cycle_speed[x];
    running_average = running_average / CHECK_CYCLE;

    for (int b = 0; b < CHECK_CYCLE; b++)
    { // check if the readings is far away from the average, then we dont change anything
      if (abs(cycle_speed[b]) < abs(running_average) * 0.8 || abs(cycle_speed[b]) > abs(running_average) * 1.2)   reject_count++;
      if (reject_count == reject)  break;
    }
    if (reject_count != reject && abs(running_average) <= 100 * rpm2rad ) { // small step, close to equilibrium
      if (running_average < -step_size)  ANGLE_REF = ANGLE_REF - step_size;
      if (running_average > step_size)   ANGLE_REF = ANGLE_REF + step_size;
    }
    if (reject_count != reject && abs(running_average) > 100 * rpm2rad) { // large step, far away from equilibrium
      if (running_average < -step_size)  ANGLE_REF = ANGLE_REF - large_step_size; //left of eq
      if (running_average > step_size)   ANGLE_REF = ANGLE_REF + large_step_size; //right of eq
    }
    running_average = 0; //reset values
    reject_count = 0;
    cycle = 0;
  }

}

void updateMotor() {
  sam_slut = micros(); // meassure time
  timer_var = micros();
  ang_err =  angle_pot();      // calculate angle error
  if (enable_kalman_filter && sensor == 2)
  {
    spf = x_post_kf(1);
    spw = x_post_kf(2);
  }
  else
  {
    spf = speed_frame();         // calculate frame speed
    spw = (((float)(((float)analogRead(SPEED_PIN) - 512)) * (2048.0 / 1024.0)) * rpm2rad); // measure flywheel speed
  }
  
  if (sensor == 1)  curr = (((k1_pot * spw + k2_pot * ang_err + k3_pot * spf)) / kt); // potentiometer controller
  if (sensor == 2)  curr = (((k1 * spw + k2 * ang_err + k3 * spf)) / kt);  // IMU controller
  if (curr >= CURRENT_MAX)  curr = CURRENT_MAX;  // if above max current set equal to max
  if (curr <= -CURRENT_MAX)  curr = -CURRENT_MAX; // if below min current set equal to min
  motor_torque = kt*curr;
  u_kf = motor_torque;
  kalman_filter_update();
  duty = (int)interpolate(curr, -CURRENT_MAX, CURRENT_MAX, freq_max, freq_min); // map the current from max to min
  //FPGA.analogWrite(PWM_PIN, map(duty, 0, 100, pow(2, bits), 0)); // set pwm of the motor
  if (!add_cycle) cycle_speed[cycle] = spw; // save the speed for the ANGLE_REF correction
  if (!add_cycle) cycle++; // add one to the array
  sam_start = timer_var;
}

//fall down procedure
void touchDown () {

  if(touchdown_start == false)
  {
    FPGA.analogWrite(PWM_PIN, map(50, 0, 100, pow(2, bits), 0)); //stop motor
    touchdown_timer= millis();
    touchdown_start = true;
  }
  ang_err =  angle_pot();

  if (abs(ang_err) > recovery - 21*3.14/180 && !touchdown_brake)
  {
      brake.write(brk - 5); //brake motor hard to get up properly 

      if(enable_touchdown_printing == true)
      {
        touchdown_measurement_timer = millis();
        while (millis() - touchdown_measurement_timer <= 75) // printing data to serial port while braking
        {        
          print_touchdown_data ();   
        }
      }
      else
      {
         delay(75);            //wait before disabling the brake
      }
      brake.write(go);      // release brake 
      touchdown_brake = true;
  }


  if (abs(ang_err) > recovery || millis() - touchdown_timer> 1200)
    { 
      FPGA.analogWrite(PWM_PIN, map(50, 0, 100, pow(2, bits), 0)); //stop motor
      digitalWrite(enable, LOW); // disable driver
    }
  else if (abs(ang_err) > 0.035)
  {
    touchdown_curr = touchdown_gain * sin(ang_err);
    if (touchdown_curr > CURRENT_MAX) touchdown_curr = CURRENT_MAX;
    if (touchdown_curr < -CURRENT_MAX) touchdown_curr = -CURRENT_MAX;
    touchdown_pwm = (int)interpolate(touchdown_curr, -CURRENT_MAX, CURRENT_MAX, freq_max, freq_min); // map the current from max to min
    FPGA.analogWrite(PWM_PIN, map(touchdown_pwm, 0, 100, pow(2, bits), 0)); // set pwm of the motor

  }  
}

//print measurement data while falling down
void print_touchdown_data()
{
    filter_setup();

    Serial.print(millis());
    Serial.print("; ");
    Serial.print(angle_pot()); //angle measurement from potentiometer
    Serial.print("; ");
    Serial.print(speed_frame()); //speed of the frame from potentiometer
    Serial.print("; ");
    Serial.println(speed_frame_imu()); //speed of the frame from IMU

}

void print_balance_data()
{
    Serial.print(millis());
    Serial.print("; ");
    Serial.print(((float)(analogRead(potPin) - 4.0) * (90.0 / 1023.0)) - 45.0); //angle measurement from potentiometer
    Serial.print("; ");
    Serial.print(speed_frame_imu()); //speed of the frame from IMU
    Serial.print("; ");
    Serial.print(AcX);
    Serial.print("; ");
    Serial.print(AcY);
    Serial.print("; ");
    Serial.print(spw);
    Serial.print("; ");
    Serial.print(motor_torque);
    Serial.print("; ");
    Serial.print(ang_err);
    Serial.print("; ");
    Serial.print(x_post_kf(0));
    Serial.print("; ");
    Serial.print(x_post_kf(1));
    Serial.print("; ");
    Serial.print(x_post_kf(2));
    Serial.print("; ");
    Serial.println(az);
}
