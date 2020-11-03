
/* Test of the packetisation of a struct into a union.
 * After this operation is the packet is transmitted over a Zigbee module configured in AT mode.
 * The reciever function counts packeges but no error detection or correction. 
*/


/* Struct for testing packet size: 
  typedef struct controlData_t {
  char cmd;
  float val1;
  float val2;
  };
*/


/* another packet structure for testing */ 
typedef struct controlData_t {
  char cmd;
  char val1;
  char val2;
  char val3;
  char val4;
  char val5;
  char val6;
  char val7;
  char val8;
  char val9;
  char val10; 
  char val11;
};

// Definition of packet union: 
typedef union ZIGBEE_Packet_t {
  controlData_t packet;
  uint8_t ZBPacket[sizeof(controlData_t)];
};

// Instantiation of 2 types of data: 
ZIGBEE_Packet_t data;
ZIGBEE_Packet_t rxdata;
#define PACKET_SIZE sizeof(data.ZBPacket)


// bare minimum functionality for transmitting packets. 
void transmit() {
  
  Serial1.write('<');
  Serial.write('<');
  for (int k = 0; k < PACKET_SIZE; k++) {
    Serial1.write(data.ZBPacket[k]);
    Serial.write(data.ZBPacket[k]);
    {
  
    }
  }
  Serial1.write('>');
  delay(2);
}




int frameid = 0;
// Function for receiving packets: 
void receive() {
  if (Serial1.available() > PACKET_SIZE+1 ) { // check serial buffer for incomming bytes.
    if (Serial1.read() == '<') {
      //Serial.write('p');
      for (int k = 0; k < PACKET_SIZE; k++) {
        rxdata.ZBPacket[k] = Serial1.read();  // Use rxdata for storing bytes from serial buffer.
       // Serial.write(rxdata.ZBPacket[k]);   //debug packet.
      }
      if (Serial1.read() == '>'){
         for (int k = 0; k < PACKET_SIZE; k++) {
       Serial.write(rxdata.ZBPacket[k]);      // print packet.
      }
     frameid ++;
     Serial.print("  count:  "); Serial.println(frameid); // frame count print. 
      }
     
     // Serial.println(rx.data.val1);
    }  
  
  }
}

int count = 0;
void setup() {
 
  Serial.begin(115200);   // serial port for debugging. 
  Serial1.begin(115200);  // Serial port for XBEE
  int k = sizeof(data.ZBPacket);
  delay(3000);            // Startup delay.
  Serial.print("Size of packet");Serial.println(k);
  pinMode(1, INPUT_PULLUP); // Button. wire on port d1 
  pinMode(LED_BUILTIN, OUTPUT); // the pcb led. 
   digitalWrite(LED_BUILTIN, LOW);
}
char temp;
void loop() {

  data.packet.val1 = '1';
  data.packet.val2 = '2';
  data.packet.val3 = '3';
  data.packet.val4 = '4';
  data.packet.val5 = '5';
  data.packet.val6 = '6';
  data.packet.val7 = '7';
  data.packet.val8 = '8';
  data.packet.val9 = '9';
  data.packet.val10 = 'a';
  data.packet.val11 = 'a';
  data.packet.cmd = 'b';
  //Serial.println("transmitted:");


 // if button clicked transmit: 
 if(digitalRead(1) == LOW){ 
 digitalWrite(LED_BUILTIN, HIGH);  
 transmit();
 delay(1000);
 digitalWrite(LED_BUILTIN, LOW);
 }


// check for received packages: 
receive();


/*
  if (Serial1.available() > PACKET_SIZE ) {
    //  microsecond();
    //  Serial.println("recieved:  ");
    for (int k = 0; k < PACKET_SIZE + 1; k++) {
      temp = Serial1.read();
      if (temp == '<') {
        Serial1.write(temp);
      }
      if (temp == 'a') {
        Serial1.write(temp);
      }
    }
    count ++;
    Serial.print("count:  "); Serial.println(count);
  }
*/
  // put your main code here, to run repeatedly:
 // int k = sizeof(data.ZBPacket);
  //delay(1000);
 // Serial.print("Size of packet");Serial.println(k);

}
