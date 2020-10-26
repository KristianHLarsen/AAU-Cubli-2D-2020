
 typedef struct controlData_t {
  char cmd;
  float val1;
  float val2;
  float val3;
  };

void printstruct(controlData_t data){
  Serial.println("Data:  ");
  Serial.print("CMD:  ");Serial.print(data.cmd);
  Serial.print(", val1:  ");Serial.print(data.val1);
  Serial.print(", val2:  ");Serial.print(data.val2);
  Serial.print(", val3:  ");Serial.println(data.val3);
}


typedef union ZIGBEE_Packet_t {
  controlData_t packet;
  uint8_t ZBPacket[sizeof(controlData_t)];
};
ZIGBEE_Packet_t data;
ZIGBEE_Packet_t rxdata;

#define PACKET_SIZE sizeof(data.ZBPacket)

// Buffer implementation: 
#include <CircularBuffer.h>
#define BUFFER_SIZE 20
controlData_t tempdata;
CircularBuffer<controlData_t, BUFFER_SIZE> rxbuffer; 



void transmit() {
  Serial1.write('<');
  for (int k = 0; k < PACKET_SIZE; k++) {
    Serial1.write(data.ZBPacket[k]);
  }
}


int badframe = 0;
int frameid = 0;

void receive() {
  if (Serial1.available() > PACKET_SIZE  ) {    
    if (Serial1.read() == '<') {
      for (int k = 0; k < PACKET_SIZE; k++) {
        rxdata.ZBPacket[k] = Serial1.read();
      }
   }
 if (rxbuffer.isFull())
    Serial.println("rx buffer overflow");
  }
  else {rxbuffer.push(rxdata.packet);}
}

// Use retrieved value use following command: 


void get_rx_data(){
    tempdata = rxbuffer.shift();
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



Serial.println("Setting values for rx packet:  ");
rxdata.packet.val1 = 33333;
rxdata.packet.val2 = 44.33;
rxdata.packet.val3 = 4.555;
rxdata.packet.cmd = 'C';

printstruct(rxdata.packet);


Serial.println("Pushing to buffer ");
rxbuffer.push(rxdata.packet);

Serial.print("Items in buffer: "); Serial.println(rxbuffer.size());
delay(1000);

Serial.println("Printing from buffer:");
get_rx_data();
printstruct(tempdata); 
delay(1000);
Serial.print("Items in buffer: "); Serial.println(rxbuffer.size());


}
