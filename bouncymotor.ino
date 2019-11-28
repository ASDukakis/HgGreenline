#include <uStepperS.h>
#include "SerialChecker.h"
#include "MilliTimer.h"

SerialChecker sc;

uStepperS stepper;
uint32_t tOld = millis();
float velocity = 100;
float maxvelocity = 300;
float angle = 0;
float distance = 0;

bool upstop = true;
bool upstopOld = true;
bool downstop = true;
bool downstopOld = true;

bool topLED = false;
bool botLED = false;

bool direc = false;   //0 for upward, 1 for downward
bool direcOld = false;

MilliTimer DebounceTimer(25);
MilliTimer ZeroTimeout(15000);

enum class states{
  IDLE,
  ZEROING,
  MOVING
};

states motorstate = states::IDLE;

void zeroing(bool reset = false); // function prototype. Function defined below.

void setup() {
  sc.init();
  sc.setMsgMinLen(1);
  // put your setup code here, to run once:
  stepper.setup();
  stepper.setMaxAcceleration(2000);
  stepper.setMaxVelocity(100);
  
  //Serial.begin(250000);
//  pinMode(2, INPUT_PULLUP);
//  pinMode(3, INPUT_PULLUP);

//  attachInterrupt(digitalPinToInterrupt(7), stopMotorTop, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(8), stopMotorBot, CHANGE);  
  Serial.println("Connected to bouncymotor.ino");
}

void loop() {
  checkEndStops();
  
  checkSerial();
  printDiagnostics();
//  if(!upstop){
//    Serial.println("Upstop hit.");
//    upstop = true;
//  }

  switch(motorstate){
    case states::IDLE:
    
      // do nothing
      break;
    case states::ZEROING:
      zeroing();
      break;
    case states::MOVING:
      // photodiode scan function
      if(stepper.getMotorState() == 0){
        motorstate = states::IDLE;
        Serial.println("boncymotor returned to idle");
      }
      break;
  }

}

void checkEndStops(){
  if(DebounceTimer.timedOut(true)){
    DebounceTimer.reset();

    upstop = digitalRead(7);
    downstop = digitalRead(8);
    
    if(upstop != upstopOld){
      stopMotorTop();
    }
    upstopOld = upstop;
 
    if(downstop != downstopOld){
      stopMotorBot();
    }
    downstopOld = downstop;
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

void zeroing(bool reset = false){
  // 1. If off the endstop, move up at speed X = maxvelocity
  // 2. Check for endstop hit. If true, Move down some steps.
  // 3. If moved off the endstop and stopped, move up at speed X = __ until top end stop hit.
  // 4. Zero the motor position counter.
  static uint8_t zStep = 1;
  if(ZeroTimeout.timedOut(true)){
    zStep = 1;
    direc = direcOld;
  }
  if(reset){
    zStep = 1;
  }
  
  upstop = digitalRead(7);
//  Serial.print(upstop);
//  Serial.print(", ");
//  Serial.print(zStep);
//  Serial.print(", ");
//  Serial.println(ZeroTimeout.timedOut());
  
  switch(zStep){
    case 1:
      // check we haven't already hit the endstop
      if(upstop == true){
        direc = 0;
        stepper.setMaxVelocity(maxvelocity);
        stepper.runContinous(0);
      }
      zStep++;
      break;
    case 2:
      if(upstop == false){
        direc = 1;
        // make motor start moving 500 steps downwards
        stepper.setMaxVelocity(50);
        stepper.moveSteps(12000);
        ZeroTimeout.reset();
        zStep++;
      }
      break;
    case 3:
      if((stepper.getMotorState() == 0) && (upstop == true)){
        direc = 0;
        stepper.setMaxVelocity(25);
        stepper.moveSteps(-20000);
        ZeroTimeout.reset();
        zStep++;      
      }
      break;
    case 4:
      if(upstop == false){
        stepper.stop(HARD);
        stepper.encoder.setHome();
        ZeroTimeout.reset();
        zStep = 1;
        direc = 1;
        motorstate = states::IDLE;
        Serial.println("Zeroing complete.");
      }
      break;
  }
  
}

void stopMotorTop(){
  if(direc == 0){
    stepper.stop(HARD);
    topLED = true;
    if(motorstate == states::MOVING){
      motorstate = states::IDLE;
    }
  }
  topLED = false;
}


void stopMotorBot(){
  if(direc == 1){
    stepper.stop(HARD);
    botLED = true;     //change to digitalwrite later?
    if(motorstate == states::MOVING){
      motorstate = states::IDLE;
    }
  }
  botLED = false;
}

void checkSerial(){
  if(sc.check()){
    if     (sc.contains("id")){ // get id
      Serial.println("BM");
    }
    else if(sc.contains("0")){  // stop
      stepper.stop(HARD);
      if(motorstate == states::ZEROING){
        direc = direcOld;
        }
//      stepper.moveSteps(15000);
      Serial.println("Motor Stopped");
      motorstate = states::IDLE;
    }
    else if(sc.contains("sv")){ // set speed
      if(motorstate != states::ZEROING){
        if(sc.toInt16() >= 25 && sc.toInt16() <= maxvelocity){
          velocity = sc.toInt16(); 
          if(motorstate == states::MOVING){
            stepper.setMaxVelocity(velocity);
          }
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
          velocity += 25;
          if(motorstate == states::MOVING){
            stepper.setMaxVelocity(velocity);
          }
        }
      }
      else if(motorstate == states::ZEROING){
        Serial.println("Please do not change speed during zeroing!");
      }
    }
    else if(sc.contains("vd")){ // set down (nudge down) the velocity
      if(motorstate != states::ZEROING){
        if(velocity <= 25){
          Serial.println("Min velocity reached");        
        }
        else{
          velocity += -25;
          if(motorstate == states::MOVING){
            stepper.setMaxVelocity(velocity);
          }
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
      Serial.print("Microstep: ");
      Serial.println(stepper.driver.getPosition());
    }
    else if(sc.contains("ze")){ // zero the stage to top microswitch
      direcOld = direc;
      zeroing(true);
      motorstate = states::ZEROING;
      Serial.println("Zeroing...");
    }
    else if(sc.contains("k")){  // toggle run/stop
//      upstop = digitalRead(7);
      if(motorstate == states::IDLE){
        //if topstop hit and td = up OR if botstop hit and td = down, print NO, else...
        if(upstop == 0 && direc == false){
          Serial.println("Please change direction before moving!");
        }
        else if(downstop == 0 && direc == true){
          Serial.println("Please change direction before moving!");
        }
        else{
          motorstate = states::MOVING;
          stepper.setMaxVelocity(velocity);
          stepper.runContinous(direc);
          //Serial.println(direc);
        }
      }
      else if(motorstate == states::MOVING || motorstate == states::ZEROING){
        if(motorstate == states::ZEROING){
          direc = direcOld;
          }
        stepper.stop(HARD);
        motorstate = states::IDLE;
      }
    }
    else if(sc.contains("td")){ // toggle direction
      direc = !direc;
      Serial.print("direction set to: ");
      if(direc == true){
        Serial.println("down"); 
      }
      else{
        Serial.println("up");
      }
      if(motorstate == states::MOVING){
        stepper.stop(HARD);
        stepper.runContinous(direc);
      }
    }
    else if(sc.contains("su")){ // set direc to up
      if(direc == 1){
        direc = 0;
        Serial.println("direction set to: up");
        if(motorstate == states::MOVING){
          stepper.stop(HARD);
          stepper.runContinous(direc);
        }
      }
      else if(direc == 0){
        Serial.println("Already moving upward.");
      }
    }
    else if(sc.contains("sd")){ // set direc to down
      if(direc == 0){
        direc = 1;
        Serial.println("direction set to: down");
        if(motorstate == states::MOVING){
          stepper.stop(HARD);
          stepper.runContinous(direc);
        }
      }
      else if(direc == 1){
        Serial.println("Already moving downward.");
      }
    }
    else if(sc.contains("ms")){
      motorstate = states::MOVING;
      if(direc == 0){
        stepper.moveSteps(-51200);
      }
      else if(direc == 1){
        stepper.moveSteps(51200);
      }
    }
  }
}
