#include "Arduino.h"
#include "Debounced_DigitalRead.h"
#include "PacketSerial_Modified.h"



// TODO -- Consider adding something for barrel jack detection
//         Currently don't utilize and can't really think of how it might be helpful

#define BAUD_RATE 921600

#define SERIAL1_TX 0
#define SERIAL1_RX 1
#define SERIAL2_TX 4
#define SERIAL2_RX 5

#define PIO_1_TX 6
#define PIO_1_RX 7
#define PIO_2_TX 8
#define PIO_2_RX 9
#define PIO_3_TX 12
#define PIO_3_RX 13
#define PIO_4_TX 14
#define PIO_4_RX 15

#define NUM_SERIAL_OBJECTS 6
#define FIFO_SIZE 2048

uint8_t decodedInputBuffer[FIFO_SIZE];
uint8_t outputBuffer[FIFO_SIZE];

SerialPIO serialPIOInstances[4] = {
  SerialPIO(PIO_1_TX, PIO_1_RX, FIFO_SIZE),
  SerialPIO(PIO_2_TX, PIO_2_RX, FIFO_SIZE),
  SerialPIO(PIO_3_TX, PIO_3_RX, FIFO_SIZE),
  SerialPIO(PIO_4_TX, PIO_4_RX, FIFO_SIZE)
};


HardwareSerial* serialObjects[NUM_SERIAL_OBJECTS];

Debounced_DigitalRead picoConnectionSensor[NUM_SERIAL_OBJECTS] = {
  Debounced_DigitalRead(2, INPUT_PULLUP),
  Debounced_DigitalRead(3, INPUT_PULLUP),
  Debounced_DigitalRead(17, INPUT_PULLUP),
  Debounced_DigitalRead(10, INPUT_PULLUP),
  Debounced_DigitalRead(11, INPUT_PULLUP),
  Debounced_DigitalRead(16, INPUT_PULLUP)
};

Debounced_DigitalRead barrelJackConnectionSensor(28, INPUT_PULLUP);

#define BUILTIN_LED 25

#define PICO_DESIGNATOR_CODE_MASK 0b01110000
#define PICO_DESIGNATOR_CODE_OFFSET_AMOUNT 4

#define ID_MSG_CODE 0b00001111

void debug(const char*);
void debug(const char*, uint8_t);
void hubPing();

bool serialConnected = false;
void setup() {
  pinMode(BUILTIN_LED, OUTPUT);
  Serial.begin(BAUD_RATE);
  Serial1.setTX(SERIAL1_TX);
  Serial1.setRX(SERIAL1_RX);
  Serial2.setTX(SERIAL2_TX);
  Serial2.setRX(SERIAL2_RX);
  Serial1.setFIFOSize(FIFO_SIZE);
  Serial2.setFIFOSize(FIFO_SIZE);
  serialObjects[0] = &Serial1;
  serialObjects[1] = &Serial2;
  for(int i = 2; i < 6; i++) serialObjects[i] = &serialPIOInstances[i-2];
  for(int i = 0; i < NUM_SERIAL_OBJECTS; i++){
    picoConnectionSensor[i].begin();
    serialObjects[i]->begin(BAUD_RATE);
  }
  rp2040.wdt_begin(1000);
}
void handleUnitySerialInputs();
void handlePicoSerialInputs();

void loop(){
  rp2040.wdt_reset();
  if(serialConnected != Serial){
    serialConnected = !serialConnected;
    if(serialConnected) debug("hub connected");
  }
  // hubPing();
  for(int i = 0; i < NUM_SERIAL_OBJECTS; i++){
    picoConnectionSensor[i].update();
  }
  // while(Serial.available()){
  //   serialObjects[5]->write(Serial.read());
  // }
  // while(serialObjects[5]->available()){
  //   Serial.write(serialObjects[5]->read());
  // }
  // return;


  handleUnitySerialInputs();
  handlePicoSerialInputs();
} 

void clearIncomingBuffer(HardwareSerial*, int = 20);

void handleUnitySerialInputs(){
  if(!Serial.available()) return;

  
  uint32_t timeoutTimer = millis();

  while(true){
    bool timedOut = false;
    uint8_t firstByte = Serial.read();
    if(firstByte == 0){
      if(Serial.available()) continue;
      else return;
    }
    if(firstByte == 1){ // means the config byte is 0. Would not happen unless we want to be able to send the same packet (aside from ID request) to all 6 Picos. Is an error for now.
      // just ignore inputs until 0 (or timeout) then reset
      debug("config byte from Unity is 0");
      clearIncomingBuffer(&Serial);
      if(Serial.available()) continue;
      else return;
    }

    uint8_t configByte = 0;
    // wait for config byte
    while(true){
      if(Serial.available()){
        configByte = Serial.read();
        break;
      }
      if(millis() - timeoutTimer > 5){
        timedOut = true;
        debug("timeout waiting for config byte from Unity");
        clearIncomingBuffer(&Serial);
        if(Serial.available()) break;
        else return;
      }
    }
    if(timedOut) continue;
    
    timeoutTimer = millis();
    // if id msg, send to all
    if(configByte == 0b00001111){
      for(int i = 0; i < NUM_SERIAL_OBJECTS; i++){
        if(!picoConnectionSensor[i].read()) continue;
        serialObjects[i]->write(firstByte);
        uint8_t newConfigByte = configByte + ((i + 1) << 4);
        serialObjects[i]->write(newConfigByte);
      }
      timeoutTimer = millis();
      while(true){
        if(millis() - timeoutTimer > 10){
          timedOut = true;
          debug("Unity packet timeout (ID)");
          clearIncomingBuffer(&Serial);
          if(Serial.available()) break;
          else return;
        }
        if(Serial.available()){
          uint8_t _nextByte = Serial.read();
          for(int i = 0 ; i < NUM_SERIAL_OBJECTS; i++){
            if(!picoConnectionSensor[i].read()) continue;
            serialObjects[i]->write(_nextByte);
          }
          if(_nextByte == 0){
            if(Serial.available()) continue;
            else return;
          }
        }
      }
      if(timedOut) continue;
    }

    // otherwise, get designator code and send to that pico
    uint8_t designatorCode = ((configByte & PICO_DESIGNATOR_CODE_MASK) >> 4);
    timeoutTimer = millis();

    if(designatorCode > 6 || designatorCode == 0){ // shouldn't happen?
      
      debug("invalid designator from Unity", designatorCode);
      clearIncomingBuffer(&Serial);
      if(Serial.available()) continue;
      else return;
    }
    uint8_t serialObjectIndex = designatorCode - 1;
    serialObjects[serialObjectIndex]->write(firstByte);
    serialObjects[serialObjectIndex]->write(configByte);
    while(true){
      if(millis() - timeoutTimer > 40){
        debug("Unity timeout (packet)");
        timedOut = true;
        clearIncomingBuffer(&Serial);
        if(Serial.available()) break;
        else return;
      }
      int availableBytes = Serial.available();
      for(int i = 0; i < availableBytes; i++){
        uint8_t _nextByte = Serial.read();
        serialObjects[serialObjectIndex]->write(_nextByte);
        if(_nextByte == 0){
          if(Serial.available()) break;
          else return;
        }
      }
    }
  }
}


void handlePicoSerialInputs(){
  for(int i = 0; i < NUM_SERIAL_OBJECTS; i++){
    if(!picoConnectionSensor[i].read()) continue;
    if(!serialObjects[i]->available()) continue;

    uint8_t firstByte = serialObjects[i]->read();
    // 0 means we start with termination byte which we don't want, 1 means config byte is 0, which shouldn't ever be the case
    if(firstByte == 0) continue;
    if(firstByte == 1){
      debug("err: config byte is 0", i + 1);
      clearIncomingBuffer(serialObjects[i]);
      continue;
    }

    uint32_t timeoutTimer = millis();
    Serial.write(firstByte); // initial zero offset for COBS
    if(Serial.available() || Serial.peek() != -1){
      uint8_t configByte = Serial.read();
      uint8_t designatorCode = configByte & PICO_DESIGNATOR_CODE_MASK;
      if(designatorCode != (i + 1)){
        configByte = (configByte & 0b10001111) + ((i + 1) << 4);
      }
      Serial.write(configByte);
    }
    timeoutTimer = millis();
    while(true){
      int availableBytes = serialObjects[i]->available();
      if(millis() - timeoutTimer > 20){
        debug("packet receive timeout", i + 1);
        break;
      }
      if(!availableBytes){
          if(serialObjects[i]->peek() != -1){ // if "available" doesn't reflect the buffer (weird PIO quirk)
              availableBytes = 1; 
          }
          // else continue; // break here..? clear incoming buffer?
      }
      // delayMicroseconds(40);
      bool packetFinished = false;
      for(int b = 0; b < availableBytes; b++){
        // delayMicroseconds(10);
        uint8_t _newByte = serialObjects[i]->read();
        Serial.write(_newByte);
        if(_newByte == 0){ // 0 is the end of packet marker
          packetFinished = true;
          break;
        }
      }
      if(packetFinished) break;
    }
  }
}

void clearIncomingBuffer(HardwareSerial* _serialObject, int timeoutLength){
  uint32_t timer = millis();
  while(true){
    if(millis() - timer > timeoutLength) return;
    if(_serialObject->available() && _serialObject->read() == 0) return;
  }
  debug("Cleared buffer");
}

char pingBuffer[8] = {'p', 'i', 'n', 'g', ':', ' ', ' ', '\0'};
void hubPing(){
  static uint8_t pingIncrementor = 0;
  static uint32_t pingTimer = 0;
  if(millis() - pingTimer < 2000) return;
  pingTimer = millis();
  pingBuffer[6] = pingIncrementor++;
  debug(pingBuffer);
}

void debug(const char* msg, uint8_t infoByte){
  uint8_t msgLength = strlen(msg);
  uint8_t bufferAndSingleByte[msgLength + 1 + 4] = {'(', infoByte, ')', ' '}; // + 1 here b/c ports are zero indexed
  for(int i = 0; i < msgLength + 1; i++){ // + 1 so we include null terminators
    bufferAndSingleByte[i+4] = msg[i];
  }
  debug((char*)bufferAndSingleByte);
} 

void debug(const char* msg){
  static uint8_t debugOutputBuffer[131] = {0b10000000, 0, 0, 0}; // capped at 127 characters
  debugOutputBuffer[1] = 0;
  uint8_t msgLength = strlen(msg);
  debugOutputBuffer[3] = msgLength - 1;
  for(int i = 0; i < msgLength; i++)
  {
    debugOutputBuffer[1] += (uint8_t)msg[i];
    debugOutputBuffer[i + 4] = (uint8_t)msg[i];
    if(i+4 >= 131) break;
  }
  _packetSerialSend(&Serial, debugOutputBuffer, msgLength + 4);
}