#include "Arduino.h"
#include "Debounced_DigitalRead.h"


// TODO -- consider adding very short delay after packet send, if helpful

// TODO -- Consider adding something for barrel jack detection and pico connection detection
//         Currently don't utilize and can't really think of how it might be helpful
//         Pins are designated and debounced digital read objects created, just not used or updating
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
  for(int i = 0; i < NUM_SERIAL_OBJECTS; i++){
    picoConnectionSensor[i].update();
  }
  handleUnitySerialInputs();
  handlePicoSerialInputs();
}

void handleUnitySerialInputs(){
  if(!Serial.available()) return;

  uint32_t timeoutTimer = millis();

  byte firstByte = Serial.read();

  if(firstByte == 1){ // means the config byte is 0. Would not happen unless we want to be able to send the same packet (aside from ID request) to all 6 Picos. Is an error for now.
    // just ignore inputs until 0 (or timeout) then reset
    // add debug msg?
    while(millis() - timeoutTimer < 20){
      if(Serial.available()){
        if(Serial.read() == 0){
          return;
        }
        // delayMicroseconds(10);
      }
    }
    return;
  }

  // wait for config byte
  byte configByte = 0;
  while(true){
    if(Serial.available()){
      configByte = Serial.read();
      break;
    }
    if(millis() - timeoutTimer > 5){
      // debug
      return;
    }
  }

  // if id msg, send to all
  if(configByte == 0b00001111){ // (if it equals 0b10001111, then it's a ping)
    for(int i = 0; i < NUM_SERIAL_OBJECTS; i++){
      if(!picoConnectionSensor[i].read()) continue;
      serialObjects[i]->write(firstByte);
      byte newConfigByte = configByte + ((i + 1) << 4);
      serialObjects[i]->write(newConfigByte);
    }
    timeoutTimer = millis();
    while(true){
      if(millis() - timeoutTimer > 10){
        // debug
        return;
      }
      if(Serial.available()){
        byte _nextByte = Serial.read();
        for(int i = 0 ; i < NUM_SERIAL_OBJECTS; i++){
          if(!picoConnectionSensor[i].read()) continue;
          serialObjects[i]->write(_nextByte);
        }
        if(_nextByte == 0) return;
      }
    }
  }

  
  // otherwise, get designator code and send to that pico
  byte designatorCode = ((configByte & PICO_DESIGNATOR_CODE_MASK) >> 4) - 1;
  timeoutTimer = millis();

  if(designatorCode > 5){ // shouldn't happen?
    // debug
    while(millis() - timeoutTimer < 20){
      if(Serial.available()){
        if(Serial.read() == 0){
          return;
        }
        // delayMicroseconds(10);
      }
    }
  }
  serialObjects[designatorCode]->write(firstByte);
  serialObjects[designatorCode]->write(configByte);
  while(true){
    if(millis() - timeoutTimer > 40){
      // debug
      return;
    }
    if(Serial.available()){
      byte _nextByte = Serial.read();
      serialObjects[designatorCode]->write(_nextByte);
      if(_nextByte == 0) return; // finished sending packet
      // delayMicroseconds(10);
    }
  }
}

void handlePicoSerialInputs(){
  for(int i = 0; i < NUM_SERIAL_OBJECTS; i++){
    if(!picoConnectionSensor[i].read()) continue;
    if(!serialObjects[i]->available()) continue;
    uint32_t timeoutTimer = millis();
    while(true){
      int availableBytes = serialObjects[i]->available();
      // check for weird PIO quirk
      if(millis() - timeoutTimer > 20){
        // Debug();
        break;
      }
      if(!availableBytes){
          if(serialObjects[i]->peek() != -1){ // if "available" doesn't reflect the buffer
              availableBytes = 1; 
          }
          else continue;
      }
      // delayMicroseconds(40);
      for(int b = 0; b < availableBytes; b++){
        // delayMicroseconds(10);
        uint8_t _newByte = serialObjects[i]->read();
        Serial.write(_newByte);
        if(_newByte == 0){ // 0 is the end of packet marker
            break;
        }
      }
    }
  }
}

void debug(byte* msg, byte msgSize){
  
}