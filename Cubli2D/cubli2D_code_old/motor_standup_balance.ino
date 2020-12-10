void state_machine()
{  
  if (sensor == 1 || sensor == 2) { // If switch is switched to IMU or POT  
    
    if (cubli_state == 'D') fallen_cubli_check(); // Change to L/R/C the first time we turn on the system.
    
    if(tempdata.cmd != 'D'){                   // if the other Cubli is not OFF 

// ************************************************** STATE MACHINE - BOTH SWITCHES ARE ON *************************************************** //

      if(cubli_state == 'L' || cubli_state == 'R') // If cubli is lying down and both switches are ON
      {
        // stop the motor, brake the wheel and change state to 'S'
        wheel_brake();        
        cubli_state = 'S';
        velocity_timer = millis();
      }
      else if((cubli_state == 'S') || (cubli_state =='V' && (tempdata.cmd == 'S' || tempdata.cmd == 'L' || tempdata.cmd == 'R'))) // If the Cubli needs to accelerate/keep the velocity for startup
      {
        // accelerate wheel and check for velocity. Change to 'V' when velocity is reached and switch back to 'S' if you lose it       
        digitalWrite(enable, HIGH);
        startup_speed_control();
        startup_velocity_check();       
      }
      else if(cubli_state =='V' && (tempdata.cmd == 'V' || tempdata.cmd == 'B' || tempdata.cmd == 'C')) // If both Cublis are ready to brake and stand up
      {
        cubli_state = 'B';      
        transmit(cubli_state, false);      
      }
      if(cubli_state == 'B')
      {
        // update motor for 1 second to stabilise, set all initial parameters and then change state to 'C',
        hard_brake();
        touchdown_start = true;
        stabilise_setup();
        cubli_state = 'C';
      }
      else if(cubli_state == 'C') 
      {
        // update motor. Change to 'L' or 'R' if cubli falls
        digitalWrite(enable, HIGH); //enable driver for writing
        if (micros() - timer_var >= samp_period)
        {
          updateMotor(); // if we have waited the sampling time.
          balancePoint();
          fallen_cubli_check();
        } 
      }
      touchdown_start = true;
      
// ************************************************** STATE MACHINE - AT LEAST ONE SWITCH IS OFF *************************************************** //    
    } else shut_down(); //if the other Cubli is switched off
  } else shut_down(); //if this Cubli is switched off
}


// ************************************************** SUBPROGRAMS ********************************************************************************* // 
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


void touchdown_slowdown()
{
  sensor = 1;
  ang_err =  angle_pot();
  if (abs(ang_err) > 0.75)
  { 
    FPGA.analogWrite(PWM_PIN, map(50, 0, 100, pow(2, bits), 0)); //stop motor
    digitalWrite(enable, LOW); // disable motor driver
    touchdown_start = false; // disable touchdown_start function
    fallen_cubli_check();
    for (int o = 0; o < 11; o++)  // function for ABS braking
    { 
      brake.write(brk + 3);
      delay(30);
      brake.write(go);
      delay(50);
    }
    brake.write(brk); // normal braking
    delay(300);
    brake.write(go); // release brake
    sensor = ogsens;
  }
  else if (abs(ang_err) > 0.035)
  {
    curr = -touchdown_gain * sin(ang_err);
    if (curr > CURRENT_MAX) curr = CURRENT_MAX;
    if (curr < -CURRENT_MAX) curr = -CURRENT_MAX;
    duty = (int)interpolate(curr, -CURRENT_MAX, CURRENT_MAX, freq_max, freq_min); // map the current from max to min
    FPGA.analogWrite(PWM_PIN, map(duty, 0, 100, pow(2, bits), 0)); // set pwm of the motor
  }
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

void debug_states()
{
  Serial.print("Cubli state: ");
  Serial.write(cubli_state);
  Serial.print("----");
  Serial.print("Other Cubli state: ");
  Serial.write(tempdata.cmd);
  Serial.println("");
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

void startup_speed_control()
{
  sensor = 1;
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
  sensor = ogsens;
}

void abs_braking()
{
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
}

void startup_velocity_check()
{
  if (millis() - velocity_timer > velocity_timer_threshold) // Ensures that the state is not switched to 'V' before a certain amount of time
  {
    if (abs(spw) > (abs(spw_ref)- spw_tolerance) || abs(spw) < (abs(spw_ref) + spw_tolerance)) cubli_state = 'V'; // switch to state V is velocity is sufficient for start up
    else cubli_state = 'S'; // switch to state S is velocity is not sufficient for start up
  }
}

void stabilise_setup()
{
    int stabilise_timer = micros();
    add_cycle = true;     //Disable ANGLE_REF corrections
    sensor = 1;
    while (micros() - stabilise_timer < 1000000) //wait for system to be stable before giving the IMU control
    {   
      updateMotor();
      delay(10);
    }
    filter_setup();
    add_cycle = false;     //ENABLE ANGLE_REF corrections
    sam_start = micros();  //Reset timing parameter since last reading
    timer_var = micros();  // --//--
    sensor = ogsens;
}

void fallen_cubli_check()
{
  cubli_state = 'P';
  sensor = 1;
  if (abs(angle_pot()) > recovery) // if we are down
  {  
    if (angle_pot() > recovery) cubli_state = 'L';
    else cubli_state = 'R';
  }
  else cubli_state = 'C';
  sensor = ogsens;
}

void wheel_brake()
{
  FPGA.analogWrite(PWM_PIN, map(50, 0, 100, pow(2, bits), 0)); //stop motor
  delay(200);        //Let it fall completely
  abs_braking();
  brake.write(brk); // normal braking
  delay(300);
  brake.write(go); // release brake
}

void hard_brake()
{
  brake.write(brk - 5); // brake motor hard to get up properly
  delay(75);            // wait before disabling the brake
  brake.write(go);      // release brake
}

void shut_down()
{
   // call the shutdown procedure
  if(touchdown_start == true) touchdown_slowdown(); // if touchdown_start is set to true, call the shutdown function
  else digitalWrite(enable, LOW);
  time_last = 0; // reset
  timer_var = 0; // reset time
  time_now = 0;  // reset  
}
