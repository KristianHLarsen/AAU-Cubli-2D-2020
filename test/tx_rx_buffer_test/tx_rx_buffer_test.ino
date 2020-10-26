
typedef struct controlData_t {
  char cmd;
  float val1;
  float val2;
  float val3;
};

void printstruct(controlData_t data) {
  Serial.println("Data:  ");
  Serial.print("CMD:  "); Serial.print(data.cmd);
  Serial.print(", val1:  "); Serial.print(data.val1);
  Serial.print(", val2:  "); Serial.print(data.val2);
  Serial.print(", val3:  "); Serial.println(data.val3);
}


typedef union ZIGBEE_Packet_t {
  controlData_t packet;
  uint8_t ZBPacket[sizeof(controlData_t)];
};
ZIGBEE_Packet_t txdata;
ZIGBEE_Packet_t rxdata;

#define PACKET_SIZE sizeof(txdata.ZBPacket)

// Buffer implementation:
#include <CircularBuffer.h>
#define BUFFER_SIZE 20
controlData_t tempdata;
CircularBuffer<controlData_t, BUFFER_SIZE> rxbuffer;



void transmit() {
  Serial1.write('<');
  for (int k = 0; k < PACKET_SIZE; k++) {
    Serial1.write(txdata.ZBPacket[k]);
  }
}


int badframe = 0;
int frameid = 0;

int receive() {
  if (Serial1.available() > PACKET_SIZE  ) {
    if (Serial1.read() == '<') {
      for (int k = 0; k < PACKET_SIZE; k++) {
        rxdata.ZBPacket[k] = Serial1.read();
      }
    rxbuffer.push(rxdata.packet);
    }
}
}
// Use retrieved value use following command:


void get_rx_data() {
  tempdata = rxbuffer.shift(); // get packet from buffer.
}


// Sample period time setting.
unsigned long t0 = 0;
const unsigned long ts = 10; // time for each transmisssion:
int count = 0;
unsigned long tstart = 0; //test vars time keeping
unsigned long tstop = 0; //test vars time keeping
bool flag_time = 0;

int N = 4; // used for checking the number of elements in the buffer.
void setup() {

  Serial.begin(115200);
  Serial1.begin(115200);
  int k = sizeof(txdata.ZBPacket);
  delay(3000);
  Serial.print("Size of packet"); Serial.println(k);
  pinMode(1, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(3, OUTPUT); // transmission begin pin goes high when a transmission starts.
  pinMode(4, OUTPUT); // when a packet is received pin goes high.
  digitalWrite(LED_BUILTIN, LOW);
  t0 = micros();
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
}
char temp;
void loop(){

  // set tx packet:

  //  Serial.println("Setting values for rx packet:  ");
   txdata.packet.val1 = 33413.345;
   txdata.packet.val2 = 44.33;
   txdata.packet.val3 = 4.555;
   txdata.packet.cmd = 'C';

  //printstruct(txdata.packet);

  // transmission:
  if (digitalRead(1) == LOW) {
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("transmit 1 packet:");
    delay(1000);
    for(int i=0; i<N; i++){
    transmit();
    }
    //while ((t0 + ts ) > micros()) {}
    // t0 = micros();
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
  }


  if (rxbuffer.size() > N) {  // check number  of elements in the buffer.
    while (rxbuffer.isEmpty() != true) { // Print all elements in the buffer.
      Serial.print("Items in buffer: "); Serial.println(rxbuffer.size());
      delay(1000);
      Serial.println("Printing from buffer:");
      get_rx_data();
      printstruct(tempdata);
      delay(1000);
      Serial.print("Items in buffer: "); Serial.println(rxbuffer.size());
    }
    rxbuffer.clear(); // clear buffer
  }
}
