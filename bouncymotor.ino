#include <uStepperS.h>
#include "SerialChecker.h"
#include "MilliTimer.h"

SerialChecker sc;

uStepperS stepper;
uint32_t tOld = millis();
float velocity = 100;
float maxvelocity = 3000;
float angle = 0;
float distance = 0;

bool upstop = true;
bool downstop = true;

bool topLED = false;
bool botLED = false;

bool dir = false;   //0 for upward, 1 for downward

MilliTimer DebounceTimer(25);
MilliTimer ZeroTimeout(20000);

enum class states{
  IDLE,
  ZEROING,
  MOVING
};

states motorstate = states::IDLE;

void setup() {
  sc.init();
  sc.setMsgMinLen(1);
  // put your setup code here, to run once:
  stepper.setup();
  stepper.setMaxAcceleration(2000);
  stepper.setMaxVelocity(100);
  
  //Serial.begin(250000);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(2), stopMotorTop, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), stopMotorBot, CHANGE);  
  Serial.println("Connected to bouncymotor.ino");
}

void loop() {
  checkSerial();
  printDiagnostics();
  

  switch(motorstate){
    case states::IDLE:
      // do nothing
      break;
    case states::ZEROING:
      zeroing();
      break;
    case states::MOVING:
      // photodiode scan function
      break;
  }

}

void printDiagnostics(){
  uint32_t t = millis();
  if(t - tOld >= 250){
    tOld = t;
//    Serial.print(digitalRead(2));
//    Serial.print(" ");
//    Serial.println(digitalRead(3));
//    Serial.print("Pos: ");
//    Serial.println(stepper.encoder.getAngle());
  }
}

void zeroing(){
  // 1. If off the endstop, move up at speed X = 3000
  // 2. Check for endstop hit. If true, Move down 500 steps.
  // 3. If moved off the endstop and stopped, move up at speed X = 100 until top end stop hit.
  // 4. Zero the motor position counter.
  static uint8_t zStep = 1;
  if(ZeroTimeout.timedOut(true)){
    zStep = 1;
  }
  upstop = digitalRead(2);
//  Serial.print(upstop);
//  Serial.print(", ");
//  Serial.print(zStep);
//  Serial.print(", ");
//  Serial.println(ZeroTimeout.timedOut());
  switch(zStep){
    case 1:
      // check we haven't already hit the endstop
      if(upstop == true){
        stepper.driver.setVelocity(maxvelocity);
        stepper.runContinous(CW);
      }
      zStep++;
      break;
    case 2:
      if(upstop == false){
        // make motor start moving 500 steps downwards
        stepper.moveSteps(-500);
        ZeroTimeout.reset();
        zStep++;
      }
      break;
    case 3:
      if((stepper.getMotorState() == 0) && (upstop == true)){
        stepper.driver.setVelocity(100);
        stepper.runContinous(CW);
        ZeroTimeout.reset();
        zStep++;      
      }
      break;
    case 4:
      if(upstop == false){
        stepper.driver.setVelocity(velocity);
        stepper.encoder.setHome();
        ZeroTimeout.reset();
        zStep = 1;
        motorstate = states::IDLE;
        Serial.println("Zeroing complete.");
      }
      break;
  }
  
}

void stopMotorTop(){
  if(DebounceTimer.timedOut(true)){
    if(dir == 1){
      stepper.stop(HARD);
      topLED = true;
      if(motorstate == states::MOVING){
        motorstate = states::IDLE;
      }
    }
    topLED = false;
    DebounceTimer.reset();
  }
}

void stopMotorBot(){
  if(DebounceTimer.timedOut(true)){
    if(dir == 1){
      stepper.stop(HARD);
      botLED = true;     //change to digitalwrite later?
      if(motorstate == states::MOVING){
        motorstate = states::IDLE;
      }
    }
    botLED = false;
    DebounceTimer.reset();
  }
}

void checkSerial(){
  if(sc.check()){
    if     (sc.contains("id")){ // get id
      Serial.println("BM");
    }
    else if(sc.contains("0")){  // stop
      stepper.stop(HARD);
      Serial.println("Motor Stopped");
      motorstate = states::IDLE;
      stepper.driver.setVelocity(velocity);
    }
    else if(sc.contains("sv")){ // set speed
      if(motorstate != states::ZEROING){
        if(sc.toInt16() > 0 && sc.toInt16() < 3000){
          velocity = sc.toInt16(); 
          stepper.driver.setVelocity(velocity);
          Serial.print("Speed has been set to: ");
          Serial.println(velocity);
        }
        else{
          Serial.println("Could not set speed to input value");
        }
      }
      else if(motorstate == states::ZEROING){
        Serial.println("Please do not change speed during zeroing!");
      }
    }
    else if(sc.contains("vu")){ // set up (nudge up) the velocity
      if(motorstate != states::ZEROING){
        if(velocity >= maxvelocity){
          Serial.println("Max velocity reached");        
        }
        else{
          stepper.driver.setVelocity(velocity += 100);
        }
      }
      else if(motorstate == states::ZEROING){
        Serial.println("Please do not change speed during zeroing!");
      }
    }
    else if(sc.contains("vd")){ // set down (nudge down) the velocity
      if(motorstate != states::ZEROING){
        if(velocity <= 100){
          Serial.println("Min velocity reached");        
        }
        else{
          stepper.driver.setVelocity(velocity += -100);
        }   
      }
      else if(motorstate == states::ZEROING){
        Serial.println("Please do not change speed during zeroing!");
      }
    }
    else if(sc.contains("gm")){ // get mode 
      Serial.print("Bouncymotor status is currently: ");
      if(motorstate == states::MOVING){
        Serial.println("controlled motion");
      }
      else if(motorstate == states::ZEROING){
        Serial.println("zeroing");
      }
      else if(motorstate == states::IDLE){
        Serial.println("idle and awaiting instruction o7");
      }
    }
    else if(sc.contains("gv")){ // get velocity 
      if(motorstate == states::ZEROING){
        Serial.println("Current velocity could not be obtained as Bouncymotor is zeroing.");
      }
      else{
        Serial.print("Current velocity is set to: ");
        Serial.println(velocity);
      }
    }
    else if(sc.contains("gp")){ // get position
      angle = stepper.encoder.getAngleMoved();
      distance = angle*8/360;
      Serial.print("Angle: ");
      Serial.println(angle);
      Serial.print("Distance: ");
      Serial.println(distance);  
    }
    else if(sc.contains("ze")){ // zero the stage to top microswitch
      motorstate = states::ZEROING;
      Serial.println("Zeroing...");
    }
    else if(sc.contains("k")){  // toggle run/stop
      stepper.driver.setVelocity(velocity);
      upstop = digitalRead(2);
      if(motorstate == states::IDLE){
        //if topstop hit and td = up OR if botstop hit and td = down, print NO, else...
        if(upstop == 0 && dir == false){
          Serial.println("Please change direction before moving!");
        }
        else if(downstop == 0 && dir == true){
          Serial.println("Please change direction before moving!");
        }
        else{
          motorstate = states::MOVING;
          stepper.runContinous(dir);
          //Serial.println(dir);
        }
      }
      else if(motorstate == states::MOVING || motorstate == states::ZEROING){
        stepper.stop(HARD);
        motorstate = states::IDLE;
      }
    }
    else if(sc.contains("td")){ // toggle direction
      dir = !dir;
      Serial.print("Direction set to: ");
      if(dir == true){
        Serial.println("down"); 
      }
      else{
        Serial.println("up");
      }
      if(motorstate == states::MOVING){
        stepper.stop(HARD);
        stepper.runContinous(dir);
      }
    }
    else if(sc.contains("su")){ // set dir to up
      if(dir == 1){
        dir = 0;
        Serial.println("Direction set to: up");
        if(motorstate == states::MOVING){
          stepper.stop(HARD);
          stepper.runContinous(dir);
        }
      }
      else if(dir == 0){
        Serial.println("Already moving upward.");
      }
    }
    else if(sc.contains("sd")){ // set dir to down
      if(dir == 0){
        dir = 1;
        Serial.println("Direction set to: down");
        if(motorstate == states::MOVING){
          stepper.stop(HARD);
          stepper.runContinous(dir);
        }
      }
      else if(dir == 1){
        Serial.println("Already moving downward.");
      }
    }
  }
}
