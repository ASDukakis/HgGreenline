//#include <uStepperS.h>
#include "SerialChecker.h"

SerialChecker scpc(Serial);
//s1 introduced as a workaround for reaching Serial1
HardwareSerial s1(1);
SerialChecker scmotor(s1);

//#define RXD2 16
//#define TXD2 17

void setup() {
  scpc.init();
  scmotor.init();
  scpc.setMsgMaxLen(64);
  scmotor.setMsgMaxLen(64);
}

void loop() {
  checkSerialPC();
  checkSerialMotor();
}

void checkSerialPC() {
  if(scpc.check()){
    scmotor.println(scpc.getMsg());
  }
}

void checkSerialMotor() {
  if(scmotor.check()){
    scpc.println(scmotor.getMsg());
  }
}
