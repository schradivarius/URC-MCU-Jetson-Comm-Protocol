#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <cstddef>
#include <cstdint>

/*
property of u of h, promise you wont read this if youre not in u of h
dont dumb here not dumb area here
*/

enum CanID {
    TEENSY = 0,
    MOTOR_FL = 1,
    MOTOR_FR = 2,
    MOTOR_ML = 3,
    MOTOR_MR = 4,
    MOTOR_BL = 5,
    MOTOR_BR = 6
};

constexpr size_t MESSAGE_SIZE = 33;
constexpr size_t DATA_SIZE = MESSAGE_SIZE - 1;

void dispatchMessage(const uint8_t* message);
CAN_message_t motorUartToCan(CanID target, const uint8_t* value);


// initialize CAN object for CAN channel 1
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

void setup() {
  Serial.begin(115200); 
  while (!Serial && millis() < 2000); 

  can1.begin();
  can1.setBaudRate(500000); 
}

void loop() {
  static uint8_t message[MESSAGE_SIZE];
  static size_t bytesReceived = 0;

  while (Serial.available() > 0) {
    const int value = Serial.read();
    if (value < 0) {
      break;
    }

    message[bytesReceived++] = static_cast<uint8_t>(value);

    if (bytesReceived == MESSAGE_SIZE) {
      dispatchMessage(message);
      bytesReceived = 0;
    }
  }
}


void dispatchMessage(const uint8_t* message) {
  const uint8_t id = message[0];
  const uint8_t* data = &message[1];

  if (id >= 1 && id <= 6) {
    motorUartToCan( (CanID)id, data);
  }

}


CAN_message_t motorUartToCan(CanID target, const uint8_t* ptr) {

  if(target<1 || target>6){
    return NULL;
  }
  
  uint32_t value = ((uint32_t)ptr[0])    |
                ((uint32_t)ptr[1] << 8)  |
                ((uint32_t)ptr[2] << 16) |
                ((uint32_t)ptr[3] << 24);

  CAN_message_t msg;

  msg.id = target; // should be in hexa but thas too hard for mine
  msg.len = 8; // can 2.0 length bub

  // gets bytes from msb to lsb
  uint8_t byte3 = (value >> 24) & 0xFF; // 0xAA
  uint8_t byte2 = (value >> 16) & 0xFF; // 0xBB
  uint8_t byte1 = (value >> 8)  & 0xFF; // 0xCC
  uint8_t byte0 =  value        & 0xFF; // 0xDD

  msg.buf[0] = byte0;
  msg.buf[1] = byte1;
  msg.buf[2] = byte2;
  msg.buf[3] = byte3;

  for (int i = 4; i <= 7; i++) {
    msg.buf[i] = 0x00;
  }

  return msg;
}