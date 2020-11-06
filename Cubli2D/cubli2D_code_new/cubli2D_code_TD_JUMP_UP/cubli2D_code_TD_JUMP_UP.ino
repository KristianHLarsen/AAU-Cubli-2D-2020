//Code for the Cubli2D
//This code is a mix of Cubli2D code from group 730(2019) and 733(2020)


//libraries
#include <stdlib.h>
#include <Servo.h>
#include <VidorFPGA.h>
#include <Wire.h>

//pin definitions
#define enable 1      //Enable motor: HIGH = on, LOW = off
#define imuIn 3       //IMU_switch PIN
#define potIn 4       //POT_switch PIN
#define potPin A1     //POT_analogValue
#define PWM_PIN 45    //Output pin for pwm on digital 5 on vidor
#define BRAKE_PIN 2   //Servo_digital pin
#define SPEED_PIN A5  //Speed from driver

//mechanical and system constants
#define CURRENT_MAX 7.5           //max current
#define recovery 40*3.14/180.0    //The Cubli has fallen
#define rpm2rad 2.0*3.14/60.0     //Conversion from RPM to Radian/s
#define rad2rpm 9.5493            //Conversion from Radian/s to RPM
#define kt 0.0335                 //Motor Constant

//self-adjusting Balance
#define CHECK_CYCLE 50
#define reject 10
#define step_size 0.1*0.01
#define large_step_size 0.1*0.1
float  ANGLE_REF = 0.5, cycle_speed[81];
int cycle = 0, reject_count = 0, running_average = 0;
bool add_cycle = false;

//servo
Servo brake;
#define brk 100  //Servo position when braking
#define go 110   //Servo position when running

//controller
//#define k1 -0.0141//-0.0316       //SpeedOfWheel
//#define k2 -9.9079//-15.6593      //AngleError
//#define k3 -0.5//-0.5             //SpeedOfFrame
#define k1 -0.0018077//-0.0316       //SpeedOfWheel
#define k2 -15.113//-15.6593      //AngleError
#define k3 -0.5296//-0.5             //SpeedOfFrame
#define k1_pot -0.001             //SpeedOfWheel - Potentiometer     
#define k2_pot -1.0               //AngleError - Potentiometer
#define k3_pot -0.1               //SpeedOfFrame - Potentiometer

//sensor
#define scale 31.8    //Scale for gyroscope (From datasheet)
#define MPU_addr 0x68 //I2C address for the IMU. (CODE WILL BE STUCK IF NOT CORRECT!)
double acc_angle_1[2] = {0, 0}, gyro_angle_1[2] = {0, 0}, comp_angle_1[2] = {0, 0}, AcX, AcY, GyZ, az, z;
int ogsens, sensor = 0 ;  // 2 for imu and 1 for potentiometer, 0 for off
float mean = 1.0, spw = 0.0, spf = 0.0, angle = 0.0, angle_last = 0.0, angle_speed = 0.0, ang_err = 0.0, curr = 0.0;

//complementary filter
double kc1, kc2, tau = 0.3471;

//PWM
#define bits 7
int freq_max = 90, freq_min = 10, freq_mid = 50, duty = 0;

//misc functionality
int samp_period = 15000;
int sam_start = 0;
int sam_slut = 0;
int timer_var = 0, time_now = 0, time_last = 0;


// standup vars
int standup_timer = 0; 
float spw_ref = 1320*rpm2rad; // standup speed reference for speed controller. RPM converted to rad/s
float k4 = 0.03; //gain for standup speed controller


// touchdown vars
boolean touchdown_start= false;
int td_spw_ref = 1400*rpm2rad; // touchdown speed reference for speed controller. RPM converted to rad/s
float k5 = 0.1; //gain for shutdown speed controller


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  IMUConfig();

  brake.attach(BRAKE_PIN);
  brake.write(go);
  for (int a = 0; a < CHECK_CYCLE; a++)  cycle_speed[a] = 0.0;

  PWMConfig();

  pinMode(enable, OUTPUT);
  pinMode(imuIn, INPUT_PULLUP);
  pinMode(potIn, INPUT_PULLUP);
  
  filter_setup();
  
  brake.write(brk);  //brake motor
  delay(1000);
  brake.write(go);   // release brake
  delay(1000);
  time_last = millis();
  sam_start = micros();
  sam_slut = micros();
}



void loop() 
{ 
  if (digitalRead(imuIn) == LOW) { // if IMU is choosen physically
    sensor = 2;
    ogsens = 2;
    samp_period = 2000; // sampling period
    touchdown_start = true;
  }
  else if (digitalRead(potIn) == LOW) { // if POT is choosen physically
    sensor = 1;
    ogsens = 1;
    samp_period = 15000; // sampling period
    touchdown_start = true;
  }
  else sensor = 0; // if system is off
  if (sensor) 
  {
    digitalWrite(enable, HIGH); //enable driver for writing
    if (micros() - timer_var >= samp_period) updateMotor(); // if we have waited the sampling time.
    balancePoint();  // check if ANGLE_REF needs correction
    stand_up();
  }
  if (!sensor)  // sytem is off 
  {
    if(touchdown_start == true) touchdown(); //if touchdown_start is set to true, call the touchdown() function
    else digitalWrite(enable, LOW); // disable motor driver
    time_last = 0; // reset
    timer_var = 0; // reset time
    time_now = 0;  // reset
  }
}
