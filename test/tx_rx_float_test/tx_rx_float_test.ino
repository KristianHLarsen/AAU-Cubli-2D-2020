
// test documenting tx and rx of packets with floats: 


// Define packet: 
  typedef struct controlData_t {
  char cmd;
  float val1;
  float val2;
  };



/*
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
*/

// Define union for transmission of packets:
typedef union ZIGBEE_Packet_t {
  controlData_t packet;
  uint8_t ZBPacket[sizeof(controlData_t)];
};
// rx packets and tx packets: 
ZIGBEE_Packet_t data;
ZIGBEE_Packet_t rxdata;
// define packet size dynamicly: 
#define PACKET_SIZE sizeof(data.ZBPacket)


// transmit function: 
void transmit() {
  
  Serial1.write('<');
  Serial.write('<');
  for (int k = 0; k < PACKET_SIZE; k++) {
    Serial1.write(data.ZBPacket[k]);
    Serial.write(data.ZBPacket[k]);
  }
  Serial1.write('>');
  Serial.write('>');
  delay(2);
}




int frameid = 0;
// receive packet: 
void receive() {
  if (Serial1.available() > PACKET_SIZE+1 ) {// check serial buffer:
    if (Serial1.read() == '<') {             // check for start charactor: 
      //Serial.write('p');                   
      for (int k = 0; k < PACKET_SIZE; k++) {// load packet into rxdata. 
        rxdata.ZBPacket[k] = Serial1.read();
       // Serial.write(rxdata.ZBPacket[k]);
      }
      if (Serial1.read() == '>'){
         for (int k = 0; k < PACKET_SIZE; k++) { // print packet on debug.
       Serial.write(rxdata.ZBPacket[k]); 
      }
     frameid ++;
     Serial.print("  count:  "); Serial.println(frameid);
      }
    Serial.print("val1  " ); Serial.println(rxdata.packet.val1); 
    Serial.print("val2  " ); Serial.println(rxdata.packet.val2);
    Serial.print("CMD  "); Serial.println(rxdata.packet.cmd);
     // Serial.println(rx.data.val1);
    }  
  
  }
}

int count = 0;
void setup() {
 
  Serial.begin(115200);   // serial port for debug
  Serial1.begin(115200);  // serial port for Xbee
  int k = sizeof(data.ZBPacket); // packet size
  delay(3000);
  Serial.print("Size of packet");Serial.println(k); // print packet size
  pinMode(1, INPUT_PULLUP); // button d1.
  pinMode(LED_BUILTIN, OUTPUT); // led for debug:
   digitalWrite(LED_BUILTIN, LOW);
}
char temp;
void loop() {
/*
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
  */
 // set test values: 
data.packet.val1 = 2444.333; 
data.packet.val2 = 3333.333;
data.packet.cmd = '2';


// transmit packet: 
 if(digitalRead(1) == LOW){ 
 digitalWrite(LED_BUILTIN, HIGH);
 Serial.println("transmitted:");  
 transmit();
 delay(1000);
 digitalWrite(LED_BUILTIN, LOW);
 }


// recieve packet: 
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
