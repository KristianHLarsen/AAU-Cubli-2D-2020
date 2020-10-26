
/*
  typedef struct controlData_t {
  char cmd;
  float val1;
  float val2;
  };
*/



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
  char val12;
};


typedef union ZIGBEE_Packet_t {
  controlData_t packet;
  uint8_t ZBPacket[sizeof(controlData_t)];
};
ZIGBEE_Packet_t data;
ZIGBEE_Packet_t rxdata;
#define PACKET_SIZE sizeof(data.ZBPacket)

// Buffer implementation: 





void transmit() {
  Serial1.write('<');
  //Serial.write('<');
  for (int k = 0; k < PACKET_SIZE; k++) {
    Serial1.write(data.ZBPacket[k]);
    //Serial.write(data.ZBPacket[k]);
  }
  //Serial1.write('>');
  //delay(2);
}


int badframe = 0;
int frameid = 0;
int receive() {
  if (Serial1.available() > PACKET_SIZE  ) {    
    if (Serial1.read() == '<') {
      //Serial.write('p');
      for (int k = 0; k < PACKET_SIZE; k++) {
        rxdata.ZBPacket[k] = Serial1.read();
        // Serial.write(rxdata.ZBPacket[k]);
      }
      digitalWrite(4, HIGH);
      //if (Serial1.read() == '>') {
      //  for (int k = 0; k < PACKET_SIZE; k++) {
      
      // Serial.write(rxdata.ZBPacket[k]);
      //}
      // check packet:
      if (check_packet() == 1) {
        frameid ++;
        // Serial.print("  count:  "); Serial.println(frameid);
        clear_rx_buff();
        return 1;
      }
      if (check_packet() == 0) {
        //Serial.print(" error  ")
        badframe++;
        clear_rx_buff();
      }
    }
    // Serial.println(rx.data.val1);
  }
}




// error handler:
int check_packet() { //Simple error detection.
  int check = 0;
  for (int k = 0; k < PACKET_SIZE; k++) {
    if (rxdata.ZBPacket[k] == data.ZBPacket[k]) {
      check ++;
    }
  }
  if (check == PACKET_SIZE) {
    return 1;
  }
  if (check != PACKET_SIZE) {
    return 0;
  }
}


// Clear buffer:
void clear_rx_buff() {
  for (int k = 0; k < PACKET_SIZE; k++) {
    rxdata.ZBPacket[k] = 0;
  }
}


// Sample period time setting.
unsigned long t0 = 0;
const unsigned long ts = 10; // time for each transmisssion:
int count = 0;
unsigned long tstart = 0; //test vars time keeping
unsigned long tstop = 0; //test vars time keeping
bool flag_time = 0;
void setup() {

  Serial.begin(115200);
  Serial1.begin(115200);
  int k = sizeof(data.ZBPacket);
  delay(3000);
  Serial.print("Size of packet"); Serial.println(k);
  pinMode(1, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(3,OUTPUT); // transmission begin pin goes high when a transmission starts.
  pinMode(4,OUTPUT); // when a packet is received pin goes high. 
  digitalWrite(LED_BUILTIN, LOW);
  t0 = micros();
  digitalWrite(3,LOW);
  digitalWrite(4,LOW);
}
char temp;
void loop() {

  data.packet.val1 = '1';
  data.packet.val2 = '1';
  data.packet.val3 = '1';
  data.packet.val4 = '1';
  data.packet.val5 = '1';
  data.packet.val6 = '1';
  data.packet.val7 = '1';
  data.packet.val8 = '1';
  data.packet.val9 = '1';
  data.packet.val10 = '1';
  data.packet.val11 = '1';
  data.packet.val12 = '1';
  //  data.packet.val12 = '1';

  data.packet.cmd = 'B';
  //Serial.println("transmitted:");

  if (digitalRead(1) == LOW) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("transmit 1000 packets:");
    delay(1000);
    digitalWrite(3,HIGH);
    for (int i = 0; i < 1000; i++) {
      transmit();
      //while ((t0 + ts ) > micros()) {}
      // t0 = micros();
    }
    digitalWrite(3,LOW);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
  }


  if (receive() == 1 && frameid == 1) {
    tstart = micros();
  }
  if (frameid + badframe > 999 && flag_time == 0) {
    tstop = micros();
    Serial.print("1000 packages recieved in:  "); Serial.print((tstop - tstart)); Serial.println(" uSeconds");
    Serial.print("Bad packets:  "); Serial.println(badframe);
    frameid = 0;
    badframe = 0;
    digitalWrite(4,LOW);
  }

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
