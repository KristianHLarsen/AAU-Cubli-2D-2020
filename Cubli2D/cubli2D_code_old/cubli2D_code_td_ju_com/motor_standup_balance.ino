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
float speed_frame_imu() 
{
  return (GyZ * 0.0174532925); 
}

void stand_up()
{

  sensor = 1; //use potentiometer for angle measurements
  
    if (abs(angle_pot()) > recovery) // if we are down
    {  
      if (angle_pot() > recovery) cubli_state = 'L';
      else cubli_state = 'R';
      transmit(cubli_state, false);
      FPGA.analogWrite(PWM_PIN, map(50, 0, 100, pow(2, bits), 0)); //stop motor
      delay(200);        //Let it fall completely
      if (abs((((float)(((float)analogRead(SPEED_PIN) - 512)) * (12000.0 / 1024.0)) * rpm2rad)) > 100.0) // if we are above a certain speed use ABS brake for not tipping over
      {
        for (int o = 0; o < 11; o++)  // function for ABS braking
        { 
          brake.write(brk + 3);
          delay(30);
          brake.write(go);
          delay(50);
        }
      }
      brake.write(brk); // normal braking
      delay(300);
      brake.write(go); // release brake
      standup_timer = millis (); // timer to give the wheel enough time to achieve reference speed
      spw = (((float)(((float)analogRead(SPEED_PIN) - 512)) * (12000.0 / 1024.0)) * rpm2rad); // measure flywheel speed
      cubli_state = 'S';

      while(tempdata.cmd != 'R' || tempdata.cmd != 'L')
      {
        receive();
        get_rx_data();
      }
      
      transmit(cubli_state, false);
      if (tempdata.cmd == 'D')  return;
      while (cubli_state != 'V' && tempdata.cmd != 'V')
      {
        receive();
        get_rx_data();
        if (angle_pot() > 0) //determine the way we fell and spin up accordingly --- negative speed direction => needs positive current
        {
          spw = (((float)(((float)analogRead(SPEED_PIN) - 512)) * (12000.0 / 1024.0)) * rpm2rad); // measure flywheel speed
          curr = -k4*(-spw_ref-spw); // Flywheel motor speed controller to achieve reference speed. Converted into current that needs to be applied to motor. 
          if (curr >= CURRENT_MAX)  curr = CURRENT_MAX;  // if above max current set equal to max
          if (curr <= -CURRENT_MAX)  curr = -CURRENT_MAX; // if below min current set equal to min
          duty = (int)interpolate(curr, -CURRENT_MAX, CURRENT_MAX, freq_max, freq_min); // map the current from max to min
          FPGA.analogWrite(PWM_PIN, map(duty, 0, 100, pow(2, bits), 0)); // set pwm of the motor
        }
        else //determine the way we fell and spin up accordingly --- positive speed direction => needs negative current
        {
          spw = (((float)(((float)analogRead(SPEED_PIN) - 512)) * (12000.0 / 1024.0)) * rpm2rad); // measure flywheel speed
          curr = -k4*(1.1*spw_ref-spw); // Flywheel motor speed controller to achieve reference speed. Converted into current that needs to be applied to motor. 1.1x multiplier as this side needs higher speed.
          if (curr >= CURRENT_MAX)  curr = CURRENT_MAX;  // if above max current set equal to max
          if (curr <= -CURRENT_MAX)  curr = -CURRENT_MAX; // if below min current set equal to min
          duty = (int)interpolate(curr, -CURRENT_MAX, CURRENT_MAX, freq_max, freq_min); // map the current from max to min
          FPGA.analogWrite(PWM_PIN, map(duty, 0, 100, pow(2, bits), 0)); // set pwm of the motor
        }
        if (abs(spw) < (abs(spw_ref)- spw_tolerance) || abs(spw) > (abs(spw_ref) + spw_tolerance)) cubli_state = 'V'; 
        else cubli_state = 'S';
        
        transmit(cubli_state, true);
      }


    // Here we are ready to apply the brake!
    cubli_state = 'B';        
    transmit(cubli_state, false);
    
    brake.write(brk - 5); //brake motor hard to get up properly
    delay(75);            //wait before disabling the brake
    brake.write(go);      // release brake
    int g = micros();
    add_cycle = true;     //Disable ANGLE_REF corrections
    while (micros() - g < 1000000) //wait for system to be stable before giving the IMU control
    {   
      updateMotor();
      delay(10);
    }
    cubli_state = 'C';
    transmit(cubli_state, false);
    filter_setup();
    add_cycle = false;     //ENABLE ANGLE_REF corrections
    sam_start = micros();  //Reset timing parameter since last reading
    timer_var = micros();  // --//--
    if (abs(ANGLE_REF) > 15.0) // if the angle ref somehow changed a lot, move it back.
    { 
      if (ANGLE_REF > 0.0) ANGLE_REF = 14.0;
      if (ANGLE_REF < 0.0) ANGLE_REF = -14.0;
    }
  }
  sensor = ogsens;         //Switch back to original sensor
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
  spf = speed_frame();         // calculate frame speed
  spw = (((float)(((float)analogRead(SPEED_PIN) - 512)) * (12000.0 / 1024.0)) * rpm2rad); // measure flywheel speed
  if (sensor == 1)  curr = (((k1_pot * spw + k2_pot * ang_err + k3_pot * spf)) / kt); // potentiometer controller
  if (sensor == 2)  curr = (((k1 * spw + k2 * ang_err + k3 * spf)) / kt);  // IMU controller
  if (curr >= CURRENT_MAX)  curr = CURRENT_MAX;  // if above max current set equal to max
  if (curr <= -CURRENT_MAX)  curr = -CURRENT_MAX; // if below min current set equal to min
  duty = (int)interpolate(curr, -CURRENT_MAX, CURRENT_MAX, freq_max, freq_min); // map the current from max to min
  FPGA.analogWrite(PWM_PIN, map(duty, 0, 100, pow(2, bits), 0)); // set pwm of the motor
  if (!add_cycle) cycle_speed[cycle] = spw; // save the speed for the ANGLE_REF correction
  if (!add_cycle) cycle++; // add one to the array
  sam_start = timer_var;
}

//Function for landing the procedure
void touchdown()
{
  sensor = 1; //use potentiometer for angle measurements 
  ang_err = angle_pot(); // measure the angle of the cubli while falling down.
  if (ang_err > 0.035 && ang_err < 0.33) // Within this range, start speeding up the wheel to NEG reference speed. 
  {
    spw = (((float)(((float)analogRead(SPEED_PIN) - 512)) * (12000.0 / 1024.0)) * rpm2rad); // measure flywheel speed in radians. 
    curr = -k5*(-td_spw_ref-spw); // Flywheel motor speed controller to achieve reference speed. Converted into current that needs to be applied to motor. 
    if (curr >= CURRENT_MAX)  curr = CURRENT_MAX;  // if above max current set equal to max
    if (curr <= -CURRENT_MAX)  curr = -CURRENT_MAX; // if below min current set equal to min
    duty = (int)interpolate(curr, -CURRENT_MAX, CURRENT_MAX, freq_max, freq_min); // map the current from max to min
    FPGA.analogWrite(PWM_PIN, map(duty, 0, 100, pow(2, bits), 0)); // set pwm of the motor
  }
  else if (ang_err < -0.035 && ang_err > -0.33)  // Within this range, start speeding up the wheel to POS reference speed. 
  {
    spw = (((float)(((float)analogRead(SPEED_PIN) - 512)) * (12000.0 / 1024.0)) * rpm2rad); // measure flywheel speed in radians. 
    curr = -k5*(1.1*td_spw_ref-spw); // Flywheel motor speed controller to achieve reference speed. Converted into current to be applied to motor. 1.1x multiplier as this side needs higher speed. 
    if (curr >= CURRENT_MAX)  curr = CURRENT_MAX;  // if above max current set equal to max
    if (curr <= -CURRENT_MAX)  curr = -CURRENT_MAX; // if below min current set equal to min
    duty = (int)interpolate(curr, -CURRENT_MAX, CURRENT_MAX, freq_max, freq_min); // map the current from max to min
    FPGA.analogWrite(PWM_PIN, map(duty, 0, 100, pow(2, bits), 0)); // set pwm of the motor
  }
  else if (abs(ang_err) > 0.33 && abs(ang_err) < 0.45) // Within this range, brake the wheel to achieve momentum for touchdown
  {
    brake.write(brk - 5); //brake motor hard to achieve momentum for touchdown
    delay (200);
    brake.write(go); // release brake
    touchdown_start = false; // disable touchdown_start function
  }
  else FPGA.analogWrite(PWM_PIN, map(50, 0, 100, pow(2, bits), 0)); //stop controlling the motor
  sensor = 0;
}

// function for if it is needed to test different speed references. 
void speed_test()
{
    digitalWrite(enable, HIGH); //enable driver for writing
    spw = (((float)(((float)analogRead(SPEED_PIN) - 512)) * (12000.0 / 1024.0)) * rpm2rad); // measure flywheel speed
    curr = -k4*(spw_ref-spw);
    Serial.print("; Speed:");
    Serial.println(spw*rad2rpm);
    if (curr >= CURRENT_MAX)  curr = CURRENT_MAX;  // if above max current set equal to max
    if (curr <= -CURRENT_MAX)  curr = -CURRENT_MAX; // if below min current set equal to min 
    duty = (int)interpolate(curr, -CURRENT_MAX, CURRENT_MAX, freq_max, freq_min); // map the current from max to min
    FPGA.analogWrite(PWM_PIN, map(duty, 0, 100, pow(2, bits), 0)); // set pwm of the motor
}
