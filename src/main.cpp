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
  for(int i = 0; i < NUM_SERIAL_OBJECTS; i++){
    picoConnectionSensor[i].update();
  }

  while(Serial.available()){
    handleUnitySerialInputs();
  }
  handlePicoSerialInputs();
} 

bool clearUnitySerialBuffer(int = 20);
bool clearPicoSerialBuffer(HardwareSerial*, int = 20);

void handleUnitySerialInputs(){
  if(!Serial.available()) return;

  uint32_t timeoutTimer = millis();

  // check for packet start byte (0)
  while(true){
    if(millis() - timeoutTimer > 30){
      // debug("0 timeout");
      return; 
    } 
    char input = Serial.read();
    if(input == 0) break; // continue on to read the packet
    debug("no 0");
    if(clearUnitySerialBuffer() && Serial.available()){ 
      continue; // try again?
    }
    else{
      return;
    }
  }
  // wait for first byte
  uint8_t firstByte;
  timeoutTimer = millis();
  while(true){
    if(millis() - timeoutTimer > 2){
      return;
    }
    if(Serial.available()){
      firstByte = Serial.read();
      if(firstByte == 0){ // means the previously read 0 was probably a packet termination byte, so getting a second 0 is okay
        delayMicroseconds(150);
        if(!Serial.available()) return;
        firstByte = Serial.read();
        if(firstByte == 0) return;
      }
      break;
    }
  }
  
  if(firstByte == 1){ // means the config byte is 0. Would not happen unless we want to be able to send the same packet (aside from ID request) to all 6 Picos. Is an error for now.
    debug("config byte from Unity is 0");
    clearUnitySerialBuffer();
    return;
  }

  timeoutTimer = millis();
  uint8_t configByte = 0;
  // wait for config byte
  while(true){
    if(Serial.available()){
      configByte = Serial.read();
      break;
    }
    if(millis() - timeoutTimer > 3){
      // debug("timeout waiting for config byte from Unity");
      clearUnitySerialBuffer();
      return;
    }
  }
  // debug("config", configByte);
  
  timeoutTimer = millis();
  // if id msg, send to all
  // modifying both the config byte and the checksum bytes for each connected device
  if(configByte == 0b00001111){
    uint8_t checksumBuffer[2];
    uint8_t checksumBufferIndex = 0;
    while(true){
      if(Serial.available()){
        checksumBuffer[checksumBufferIndex++] = Serial.read();
        if(checksumBufferIndex == 2) break;
      }
      if(millis() - timeoutTimer > 3){
        // debug("timeout waiting for checksum from Unity");
        clearUnitySerialBuffer();
        return;
      }
    }

    uint16_t _checksum = (checksumBuffer[0] & 0xFF) + (checksumBuffer[1] << 8);

    for(int i = 0; i < NUM_SERIAL_OBJECTS; i++){
      if(!picoConnectionSensor[i].read()) continue;
      uint8_t newConfigByte = configByte + ((i + 1) << 4);
      uint16_t newChecksum = _checksum - configByte + newConfigByte;
      serialObjects[i]->write((uint8_t)0);
      serialObjects[i]->write(firstByte);
      serialObjects[i]->write(newConfigByte);
      serialObjects[i]->write(newChecksum & 0xFF);
      serialObjects[i]->write(newChecksum >> 8);
    }
    timeoutTimer = millis();
    while(true){
      if(millis() - timeoutTimer > 5){
        // debug("Unity timeout (ID)");
        clearUnitySerialBuffer();
        return;
      }
      if(Serial.available()){
        uint8_t _nextByte = Serial.read();
        for(int i = 0 ; i < NUM_SERIAL_OBJECTS; i++){
          if(!picoConnectionSensor[i].read()) continue;
          serialObjects[i]->write(_nextByte);
        }
        if(_nextByte == 0){
          return;
        }
      }
    }
  }

  // otherwise, get designator code and send to that pico
  uint8_t designatorCode = ((configByte & PICO_DESIGNATOR_CODE_MASK) >> 4);
  timeoutTimer = millis();

  if(designatorCode > 6 || designatorCode == 0){ // shouldn't happen?
    // debug("invalid designator from Unity", designatorCode);
    clearUnitySerialBuffer();
    return;
  }
  uint8_t serialObjectIndex = designatorCode - 1;
  if(!picoConnectionSensor[serialObjectIndex].read()){
    debug("pico not connected", designatorCode);
    clearUnitySerialBuffer();
    return;
  }
  serialObjects[serialObjectIndex]->write((uint8_t)0);
  serialObjects[serialObjectIndex]->write(firstByte);
  serialObjects[serialObjectIndex]->write(configByte);
  while(true){
    if(millis() - timeoutTimer > 20){
      debug("Unity timeout (packet)");
      clearUnitySerialBuffer();
      return;
    }
    int availableBytes = Serial.available();
    for(int i = 0; i < availableBytes; i++){
      uint8_t _nextByte = Serial.read();
      serialObjects[serialObjectIndex]->write(_nextByte);
      if(_nextByte == 0){
        return;
      }
    }
  }
}

void handlePicoSerialInputs(){
  for(int i = 0; i < NUM_SERIAL_OBJECTS; i++){
    if(!picoConnectionSensor[i].read()) continue;
    if(!serialObjects[i]->available() && serialObjects[i]->peek() == -1) continue;



    uint32_t timeoutTimer = millis();
     // check for packet start byte (0)
    char __input = serialObjects[i]->read();
    if(__input != 0){
      debug("no 0 from pico", __input);
      clearPicoSerialBuffer(serialObjects[i]);
      continue;
    }
    // while(true){
    //   if(serialObjects[i]->read() == 0) break; // continue to read the packet
    //   if(!clearPicoSerialBuffer(serialObjects[i])) return; // if the buffer clears, try again
    //   if(millis() - timeoutTimer > 30) return; 
    // }
    uint8_t firstByte;
    timeoutTimer = millis();
    bool needToContinue = false;
    while(true){
      if(millis() - timeoutTimer > 3){
        debug("first byte timeout", i);
        needToContinue = true;
        break;
      }
      if(serialObjects[i]->available()){
        firstByte = serialObjects[i]->read();
        if(firstByte == 0){ // means the previously read 0 was probably a packet termination byte, so getting a second 0 is okay
          delayMicroseconds(150);
          if(!serialObjects[i]->available()){
            needToContinue = true;
            break;
          }
          firstByte = serialObjects[i]->read();
          if(firstByte == 0){
            needToContinue = true;
            break;
          }
        }
        break;
      }
      
    }
    if(needToContinue) continue;
    // 1 means config byte is 0, which shouldn't ever be the case
    if(firstByte == 1){
      debug("err: config byte is 0", i + 1);
      clearPicoSerialBuffer(serialObjects[i]);
      continue;
    }
    timeoutTimer = millis();
    while(!serialObjects[i]->available()){
      if(millis() - timeoutTimer > 2){
        debug("config byte timeout");
        needToContinue = true;
        break;
      }
    }
    if(needToContinue) continue;
    uint8_t configByte = serialObjects[i]->read();
    uint8_t designatorCode = configByte & PICO_DESIGNATOR_CODE_MASK;

    if(designatorCode != ((i + 1) << 4)){ // modify config byte and checksum before sending on
      uint8_t prevConfigByte = configByte;
      configByte = (configByte & 0b10001111) + ((i + 1) << 4);
      int checksumIndex = 0;
      uint8_t checksumBuffer[2];
      timeoutTimer = millis();
      while(true){
        if(serialObjects[i]->available()){
          checksumBuffer[checksumIndex++] = serialObjects[i]->read();
          if(checksumIndex == 2) break;
        }
        if(millis() - timeoutTimer > 2){
          needToContinue = true;
          break;
        }
      }
      if(needToContinue) continue;
      int newChecksum = (checksumBuffer[0] & 0xFF) + (checksumBuffer[1] << 8) - prevConfigByte + configByte;
      Serial.write((uint8_t)0);
      Serial.write(firstByte); // initial zero offset for COBS
      Serial.write(configByte);
      Serial.write(newChecksum & 0xFF);
      Serial.write(newChecksum >> 8);

    }
    else{
      Serial.write((uint8_t)0);
      Serial.write(firstByte); // initial zero offset for COBS
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
          else continue; // break here..? clear incoming buffer?
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

bool clearUnitySerialBuffer(int timeoutLength){
  uint32_t timer = millis();
  uint8_t clearedBuffer[100];
  int index = 0;
  while(true){
    if(millis() - timer > timeoutLength){
      clearedBuffer[index] = 0;
      // debug((char*)clearedBuffer, 0);
      return false;
    }
    if(Serial.available()){
      clearedBuffer[index] = Serial.read();
      if(clearedBuffer[index] == 0){
        // debug((char*)clearedBuffer, 1);
        return true;
      }
      index++;
    }
    if(Serial.available() && Serial.read() == 0) return true;
  }
  return true;
  // debug("Cleared buffer");
}

bool clearPicoSerialBuffer(HardwareSerial* _serialObject, int timeoutLength){
  uint32_t timer = millis();
  while(true){
    if(millis() - timer > timeoutLength){
      return false;
    }
    if(_serialObject->available() || _serialObject->peek() != -1){
      if(_serialObject->read() == 0) return true;
    }
  }
  return true;
  // debug("Cleared buffer");
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
  for(int i = 0; i < msgLength; i++){
    bufferAndSingleByte[i+4] = msg[i];
  }
  bufferAndSingleByte[msgLength+4] = 0; //null terminator
  debug((char*)bufferAndSingleByte);
} 

void debug(const char* msg){
  uint8_t msgLength = strlen(msg);
  if(msg[0] == '(' && msg[1] == 0){
    msgLength = 2;
    while(true){
      if(msg[msgLength] == '\0') break;
      msgLength++;
      if(msgLength >= 127) break;
    }
  }
  uint8_t debugOutputBuffer[132] = {0b10000000, 0, 0, 0, msgLength - 1}; // capped at 127 data characters
  
  uint16_t checksum = 0b10000000 + debugOutputBuffer[4];
  for(int i = 0; i < msgLength; i++)
  {
    checksum += (uint8_t)msg[i];
    debugOutputBuffer[i + 5] = (uint8_t)msg[i];
    if(i+5 >= 131) break;
  }
  debugOutputBuffer[1] = checksum & 0xFF;
  debugOutputBuffer[2] = checksum >> 8;
  Serial.write((uint8_t)0);
  _packetSerialSend(&Serial, debugOutputBuffer, msgLength + 5);
}