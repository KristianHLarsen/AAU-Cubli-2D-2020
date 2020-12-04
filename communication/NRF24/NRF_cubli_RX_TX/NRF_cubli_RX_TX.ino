#include <SPI.h>
#include "RF24.h"

byte addresses[][6] = {"1Node","2Node"};


/****************** User Config ***************************/
/***      Set this radio as radio number 0 or 1         ***/
bool radioNumber = 1;

/* Hardware configuration: Set up nRF24L01 radio on SPI bus plus pins 7 & 8 */
RF24 radio(7,8);
/**********************************************************/


// Used to control whether this node is sending or receiving
bool role = 0; // set to 1 for transmissions

/**
* Create a data structure for transmitting and receiving data
* This allows many variables to be easily sent and received in a single transmission
* See http://www.cplusplus.com/doc/tutorial/structures/
*/
struct dataStruct{
  unsigned long _micros;
  float value;
}myTXData,myRXData;




void RX(){
if(radio.available()){
  radio.read( &myRXData, sizeof(myRXData) );
  Serial.println(myRXData.value);
}
}

void TX(){
  radio.stopListening();  
  if (!radio.write( &myTXData, sizeof(myTXData) )){
       Serial.println(F("failed"));
     }
  radio.startListening();  
}

unsigned long ts = 1000000, timer = 0; // timer variables for transmissions. 

void setup() {

  Serial.begin(115200);
  radio.begin();

  // Set the PA Level low to prevent power supply related issues since this is a
 // getting_started sketch, and the likelihood of close proximity of the devices. RF24_PA_MAX is default.
  radio.setPALevel(RF24_PA_LOW);
  
  // Open a writing and reading pipe on each radio, with opposite addresses
  if(radioNumber){
    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1,addresses[0]);
  }else{
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1,addresses[1]);
  }
  myTXData.value = 1.22;
  // Start the radio listening for data
  radio.startListening();
  timer = micros();
}

void loop(){
if(role == 1) { 
if(micros() - timer >ts ){
  TX();
  myTXData.value = myTXData.value + 0.01;
  timer = micros();
}
}

RX();

  
}
