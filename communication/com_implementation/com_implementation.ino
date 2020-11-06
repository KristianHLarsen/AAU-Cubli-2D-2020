// This sketch is used for testing and devoloping an algorithm for delay estimation
// betweem the two Cubli's
// buffer doc: https://github.com/rlogiacco/CircularBuffer


//Communication:
//struct for transmitting and receiving data:
struct controlData_t {
  float val1;
  float val2;
  float val3;
};

// struct for buffering and accessing data:
struct controlData_full {
  uint8_t cmd;
  float val1;
  float val2;
  float val3;
};


// structs for delay packets
struct delay_est {
  int PacketNumber;
  float est1;
  float est2;
};

struct delay_est_full {
  uint8_t cmd;
  int PacketNumber;
  float est1;
  float est2;
};


uint8_t cmd;
char rxcmd;

union ZIGBEE_Packet_t {
  controlData_t packet;
  uint8_t ZBPacket[sizeof(controlData_t)];
};

union XBEE_PacketDelay_t {
  delay_est packet;
  uint8_t ZBPacket[sizeof(delay_est)];
};

XBEE_PacketDelay_t txdelay;
XBEE_PacketDelay_t rxdelay;

ZIGBEE_Packet_t txdata;
ZIGBEE_Packet_t rxdata;

#define PACKET_SIZE sizeof(txdata.ZBPacket)

// Buffer setup
#include <CircularBuffer.h>
#define BUFFER_SIZE 60
controlData_full tempdata;
controlData_full rxdatafull;

CircularBuffer<controlData_full, BUFFER_SIZE> rxbuffer;


// Transmit function, takes command argument which is used for
void transmit(uint8_t cmd) {
  Serial1.write(cmd);
  for (int k = 0; k < PACKET_SIZE; k++) {
    Serial1.write(txdata.ZBPacket[k]);
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
  if (Serial1.available() > PACKET_SIZE ) {
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
  tempdata = rxbuffer.shift(); // get packet from buffer.
}


void countdown() {
  Serial.println("Starting delay test in: 5s ");
  delay(1000);
  Serial.println("Starting delay test in: 4s ");
  delay(1000);
  Serial.println("Starting delay test in: 3s ");
  delay(1000);
  Serial.println("Starting delay test in: 2s ");
  delay(1000);
  Serial.println("Starting delay test in: 1s ");
  delay(1000);
}

//void packet_timeout(){

//}

int i = 4;

// Assign pins for testing purposes:
#define delay_est1_pin 1
#define delay_est2_pin 2
#define transmit_pin 3


// Parameters used for time estimation:
unsigned long tstop = 0, tdelay = 0, timer_var = 0;
unsigned long tstart[3000];
const unsigned long packet_timeout = 100000;
const unsigned long ts = 8000; // Sample time for the system:
int PacketNumber = 4;
int packetdelay = 0;

bool flag = 0;
void setup() {
  Serial.begin(500000);
  Serial1.begin(115200);
  // pinMode(delay_est1_pin, OUTPUT);
  // pinMode(delay_est2_pin, OUTPUT);
  // pinMode(transmit_pin, INPUT_PULLUP);
  PacketNumber = 0;
  ///rxbuffer.clear();
  delay(5000);
  Serial.print("Packet Size:  "); Serial.println((sizeof(delay_est) + 1));
  countdown();
  timer_var = 0;
  flag = 0;
}




void loop() {
  receive();
// Types of states:
// 'L' "im down to the left"
// 'R' "im down to the right"
// 'S'  "get ready to standup"
// 'V'  "Stand up velocity reached"
// 'B'  "stand up with the brake"
// 'C'  "Im standing up"
// 'D'  "Shutdown"
 // Inside control loop: 
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

}
