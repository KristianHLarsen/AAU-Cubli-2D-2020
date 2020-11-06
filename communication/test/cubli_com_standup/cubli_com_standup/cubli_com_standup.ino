//Code for the Cubli2D
//This code is a mix of Cubli2D code from group 730(2019) and 733(2020)


//libraries
#include <stdlib.h>
#include <Servo.h>
#include <VidorFPGA.h>
#include <Wire.h>

//Communication vars:

uint8_t cubli_state; //state of this cubli
boolean enable_com = true; // set to true to enable communication, set to false to run in individual mode
//struct for transmitting and receiving data:
struct controlData_t {
  float val1;
  float val2;
  float val3;
};

// struct for buffering and accessing data:
struct controlData_full {
  uint8_t cmd; // state of other cubli
  float val1;
  float val2;
  float val3;
};


uint8_t cmd;
char rxcmd;

union ZIGBEE_Packet_t {
  controlData_t packet;
  uint8_t ZBPacket[sizeof(controlData_t)];
};



ZIGBEE_Packet_t txdata;
ZIGBEE_Packet_t rxdata;

#define PACKET_SIZE sizeof(txdata.ZBPacket)

// Buffer setup
#include <CircularBuffer.h>
#define BUFFER_SIZE 60
controlData_full tempdata;
controlData_full rxdatafull;

CircularBuffer<controlData_full, BUFFER_SIZE> rxbuffer;
unsigned long transmit_timer = 0;
unsigned long timer_threshold = 10000; // time after which we can transfer again

void transmit(uint8_t cmd, bool timer_enable) {
  if(timer_enable)
  {
    if (micros()-transmit_timer > timer_threshold)
    {
      transmit_timer = micros();
      Serial1.write(cmd);
      for (int k = 0; k < PACKET_SIZE; k++) {
        Serial1.write(txdata.ZBPacket[k]);
        
      }
    }
  }
  else
  {
    transmit_timer = micros();
    Serial1.write(cmd);
    for (int k = 0; k < PACKET_SIZE; k++) {
      Serial1.write(txdata.ZBPacket[k]);
      
    }
  }
}

// Receive functionalities:
// Types of states:
// 'L' "im down to the left"
// 'R' "im down to the right"
// 'S'  "get ready to standup"
// 'V'  "Stand up velocity reached"
// 'B'  "stand up with the brake"
// 'C'  "Im standing up"
// 'D'  "Shutdown"

void receive() {
  while (Serial1.available() > PACKET_SIZE ) {
    uint8_t cmdtemp = Serial1.read();
    if (cmdtemp == 'L' || cmdtemp == 'R' || cmdtemp == 'S' || cmdtemp == 'V' || cmdtemp == 'B' || cmdtemp == 'C' || cmdtemp == 'D') {
      for (int k = 0; k < PACKET_SIZE; k++) {
        rxdata.ZBPacket[k] = Serial1.read();
      }
      rxdatafull.cmd = cmdtemp;
      rxdatafull.val1 = rxdata.packet.val1;
      rxdatafull.val2 = rxdata.packet.val2;
      rxdatafull.val3 = rxdata.packet.val3;
      rxbuffer.push(rxdatafull);
      cmdtemp = 0;
    }
  }
}
// get retrieved from buffer value using following command:
void get_rx_data() {
 if (rxbuffer.isEmpty() != true) { // Print 1 element from buffer.
      tempdata = rxbuffer.shift(); // get packet from buffer.
 }  
}



void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
}


uint8_t test_data;


void loop() 
{ 
  if (Serial.available() > 0){
    test_data = Serial.read();
    if(test_data == '+'){
      Serial1.print(test_data);
      Serial1.print(test_data);
      Serial1.print(test_data);
      while(Serial1.available()<2){if(Serial.available()>0)break;}
      Serial.print(Serial1.read());
      Serial.print(Serial1.read());
      Serial.println(Serial1.read());
      }
    else{  
    transmit(test_data, false);
    Serial.println(test_data);
    }
  }
}




/* 
if (micros() - timer_var >= ts) {
  // update values



  if (rxbuffer.isEmpty() != true) { // Print 1 element from buffer.
      get_rx_data();
      
      if (tempdata.cmd =='L'){} // "I'm down to the left"
      else if (tempdata.cmd =='R'){} // "I'm down to the right"
      else if (tempdata.cmd =='S'){} // "Get ready to standup"
      else if (tempdata.cmd =='V'){} // "Stand up velocity reached"
      else if (tempdata.cmd =='B'){} // "Stand up with the brake"
      else if (tempdata.cmd =='D'){} // "Shutdown"
      else if (tempdata.cmd =='C'){} // "I'm standing up"
      Serial.println("Read from buffer");
    }
    
   // control code: 
  }
  
 // Outside the control loop:
  if (rxbuffer.size() >= 4 ) {
     get_rx_data();
    }
*/
