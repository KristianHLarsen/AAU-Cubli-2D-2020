// This skecth does not work at the moment, rx function needs to be fixed.  

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
typedef union ZIGBEE_Packet_t {
  controlData_t packet;
  uint8_t ZBPacket[sizeof(controlData_t)];
};
ZIGBEE_Packet_t data;
ZIGBEE_Packet_t rxdata;
#define PACKET_SIZE sizeof(data.ZBPacket)



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
void receive() {
  if (Serial1.available() > PACKET_SIZE + 1 ) {
    if (Serial1.read() == '<') {
      //Serial.write('p');
      for (int k = 0; k < PACKET_SIZE; k++) {
        rxdata.ZBPacket[k] = Serial1.read();
        // Serial.write(rxdata.ZBPacket[k]);
      }
      if (Serial1.read() == '>') {
      /*  for (int k = 0; k < PACKET_SIZE; k++) {
          Serial.write(rxdata.ZBPacket[k]);
        {*/
        //frameid ++;
       // Serial.print("  count:  "); Serial.println(frameid);
      
      if (rxdata.packet.val1 < 2444.333 + 0.3  ||  rxdata.packet.val1 > 2444.333 - 0.3 ) {
        if (rxdata.packet.val2 < 3333.333 + 0.3  ||  rxdata.packet.val2 > 3333.333 - 0.3 ) {
          if( rxdata.packet.cmd == '2'){
            //Serial.print("val1  " ); Serial.println(rxdata.packet.val1);
            //Serial.print("val2  " ); Serial.println(rxdata.packet.val2);
            //Serial.print("CMD  "); Serial.println(rxdata.packet.cmd);
            frameid ++;
            Serial.print("  count:  "); Serial.println(frameid);
            clear_rx_buff();
          }
        }
      }
    }
      // Serial.println(rx.data.val1);
     
    }

  }
}

void clear_rx_buff() {
  for (int k = 0; k < PACKET_SIZE; k++) {
    rxdata.ZBPacket[k] = 0;
  }
}


// Sampling
unsigned long t0 = 0;
const unsigned long ts = 2;




int count = 0;
void setup() {

  Serial.begin(115200);
  Serial1.begin(115200);
  int k = sizeof(data.ZBPacket);
  delay(3000);
  Serial.print("Size of packet"); Serial.println(k);
  pinMode(1, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  t0 = millis();
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

  data.packet.val1 = 2444.333;
  data.packet.val2 = 3333.333;
  data.packet.cmd = '2';

  if (digitalRead(1) == LOW) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("transmitted:");
    transmit();
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
  }



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
 // Wait
 
while (t0 +ts) > millis()) {}
  t0 = millis();

}
