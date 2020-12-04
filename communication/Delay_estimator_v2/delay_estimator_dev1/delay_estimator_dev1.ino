// This sketch is used for testing and devoloping an algorithm for delay estimation
// betweem the two Cubli's
// buffer doc: https://github.com/rlogiacco/CircularBuffer
// For no buffer set 

const int playout_size = 1;

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
  float est3;
};

struct delay_est_full {
  uint8_t cmd;
  int PacketNumber;
  float est1;
  float est2;
  float est3;
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


delay_est_full DelayDataFull;
delay_est_full Dtempdata;
CircularBuffer<controlData_full, BUFFER_SIZE> rxbuffer;
CircularBuffer<delay_est_full, BUFFER_SIZE> rxdelaybuffer;

// Transmit function, takes command argument which is used for
void transmit(uint8_t cmd) {
  Serial1.write(cmd);
  for (int k = 0; k < PACKET_SIZE; k++) {
    Serial1.write(txdata.ZBPacket[k]);
  }
}

void DelayTransmit(uint8_t cmd) {
  Serial1.write(cmd);
  for (int k = 0; k < PACKET_SIZE; k++) {
    Serial1.write(txdelay.ZBPacket[k]);
  }
}
// Types of states:
// 'L' "im down to the left"
// 'R' "im down to the right"
// 'S'  "get ready to standup"
// 'V'  "Stand up velocity reached"
// 'B'  "stand up with the brake"
// 'C'  "Im standing up"
// 'D'  "Shutdown"
unsigned long delay_est_timer = 0;
unsigned long delay_est_timer_last = 0;
unsigned long delay_time[20] ={0};
unsigned long delay_timeout = 40000; // Delay timeout set to 20 ms.
int delay_array_handler = 0;

void receive() {
  if (Serial1.available() > PACKET_SIZE ) {
    uint8_t cmdtemp = Serial1.read();
    if (cmdtemp =='C' ) {
      // rxcmd= Serial.read();
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
    //Delay packets:
    if (cmdtemp == 'D') {
      // rxcmd= Serial.read();
      for (int k = 0; k < PACKET_SIZE; k++) {
        rxdelay.ZBPacket[k] = Serial1.read();
      }
      DelayDataFull.cmd = cmdtemp;
      DelayDataFull.PacketNumber = rxdelay.packet.PacketNumber;
      DelayDataFull.est1 = rxdelay.packet.est1;
      DelayDataFull.est2 = rxdelay.packet.est2;
      rxdelaybuffer.push(DelayDataFull);
      cmdtemp = 0;
    }
}
}
// get retrieved from buffer value using following command:
void get_rx_data() {
  tempdata = rxbuffer.shift(); // get packet from buffer.
}
void get_delay_data() {
  Dtempdata = rxdelaybuffer.shift();
}


void countdown(){
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

unsigned long delay_1 =0;
int i = 4;

// Assign pins for testing purposes:
#define delay_est1_pin 1
#define delay_est2_pin 2
#define transmit_pin 3


// Parameters used for time estimation:
unsigned long tstop = 0, tdelay = 0, timer_var = 0;
unsigned long tstart[5001];
const unsigned long packet_timeout=100000;
const unsigned long ts = 200000; // Sample time for the system:
int PacketNumber = 4;
int packetdelay=0;

bool flag = 0;
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
 // pinMode(delay_est1_pin, OUTPUT);
 // pinMode(delay_est2_pin, OUTPUT);
 // pinMode(transmit_pin, INPUT_PULLUP);
  PacketNumber = 0;
  ///rxdelaybuffer.clear();
  delay(5000);
  Serial.print("Packet Size:  ");Serial.println((sizeof(delay_est)+1));
  countdown();
  timer_var = 0;
  flag = 0;
}



void loop() {
receive();
//if (delay_1 == 0)Serial.println("Timeout");
//if (delay_1 > 0){Serial.print("delay_1:  "); Serial.println(delay_1);}


 if (micros() - timer_var >= ts) {
    timer_var = micros();
    tstart[PacketNumber] = micros();
    i++;
    txdelay.packet.PacketNumber = PacketNumber;
    //Serial.print("Packet number: "); Serial.print(PacketNumber); Serial.print(" |  ");
    PacketNumber = PacketNumber+1;
    DelayTransmit('D');

 
 /* // transmit packet back dev1
  if (rxdelaybuffer.size() >= 1) {
    if (micros() - timer_var >= ts) {
    timer_var = micros();
    if (rxdelaybuffer.isEmpty() != true) { // check if buffer is empty: if not then transmit packet.
      get_delay_data();
      txdelay.packet.PacketNumber = DelayDataFull.PacketNumber;
      DelayTransmit('D');
    }
    }
  }*/


 if (rxdelaybuffer.size() >=1 ){
 // flag =1;
    if (rxdelaybuffer.isEmpty() != true) { // Print 1 element from buffer.
     get_delay_data();
     packetdelay = Dtempdata.PacketNumber;
     //packetdelay= packetdelay;
      tstop = (micros() - tstart[packetdelay]);
      //Serial.print("Delay 2 way: ");
      Serial.println(tstop);
      //Serial.print("[us]"); Serial.print("PacketNumber: "); Serial.println(packetdelay);
    }
  }



 

 }
 
  if (rxdelaybuffer.size() >=playout_size ){
 // flag =1;
    if (rxdelaybuffer.isEmpty() != true) { // Print 1 element from buffer.
     get_delay_data();
     packetdelay = Dtempdata.PacketNumber;
     //packetdelay= packetdelay;
      tstop = (micros() - tstart[packetdelay]);
      //Serial.print("Delay 2 way: ");
      Serial.println(tstop);
      //Serial.print("[us]"); Serial.print("PacketNumber: "); Serial.println(packetdelay);
    }
  }
  
 /*if (rxdelaybuffer.size() >=8 ){
    get_delay_data();
     packetdelay = Dtempdata.PacketNumber;
      tstop = (micros() - tstart[packetdelay]);
      //Serial.print("Delay 2 way: ");
      Serial.println(tstop);
 }*/




 /*  // Print test result dev2
     get_delay_data();
     packetdelay = Dtempdata.PacketNumber;
     //packetdelay= packetdelay;
      tstop = (micros() - tstart[packetdelay]);
      //Serial.print("Delay 2 way: ");
      Serial.println(tstop);
      //Serial.print("[us]"); Serial.print("PacketNumber: "); Serial.println(packetdelay);
  }  */
  
if(i > 5000){ // Reset counter to avoid overflow.
i = 4 ;
PacketNumber = 0;
}


}
