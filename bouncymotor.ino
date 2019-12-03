#include <uStepperS.h>
#include "SerialChecker.h"
#include "MilliTimer.h"

SerialChecker scpc(Serial);
SerialChecker scesp(Serial1);

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
  scpc.init();
  scesp.init();
  scpc.setMsgMinLen(1);
  scesp.setMsgMinLen(1);
  scpc.setMsgMaxLen(64);
  scesp.setMsgMaxLen(64);
  // put your setup code here, to run once:
  stepper.setup();
  stepper.setMaxAcceleration(2000);
  stepper.setMaxVelocity(100);
  
  //Serial.begin(250000);
//  pinMode(2, INPUT_PULLUP);
//  pinMode(3, INPUT_PULLUP);

//  attachInterrupt(digitalPinToInterrupt(7), stopMotorTop, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(8), stopMotorBot, CHANGE);  
  scpc.println("Connected to bouncymotor.ino");
  scesp.println("Connected to bouncymotor.ino");
}

void loop() {
  checkEndStops();
  checkSerialPC();
  checkSerialESP();
  printDiagnostics();
//  if(!upstop){
//    scpc.println("Upstop hit.");
//    scesp.println("Upstop hit.");
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
        scpc.println("bouncymotor returned to idle");
        scesp.println("bouncymotor returned to idle");
        scpc.print("Angle: ");
        scesp.print("Angle: ");
        scpc.println(stepper.encoder.getAngleMoved());
        scesp.println(stepper.encoder.getAngleMoved());
        scpc.print("Microsteps: ");
        scesp.print("Microsteps: ");
        scpc.println(stepper.driver.getPosition());
        scesp.println(stepper.driver.getPosition());
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
//    scpc.print(digitalRead(2));
//    scpc.print(" ");
//    scpc.println(digitalRead(3));
//    scpc.print("Pos: ");
//    scpc.println(stepper.encoder.getAngle());

//    scesp.print(digitalRead(2));
//    scesp.print(" ");
//    scesp.println(digitalRead(3));
//    scesp.print("Pos: ");
//    scesp.println(stepper.encoder.getAngle());
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
//  scpc.print(upstop);
//  scpc.print(", ");
//  scpc.print(zStep);
//  scpc.print(", ");
//  scpc.println(ZeroTimeout.timedOut());

//  scesp.print(upstop);
//  scesp.print(", ");
//  scesp.print(zStep);
//  scesp.print(", ");
//  scesp.println(ZeroTimeout.timedOut());
  
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
        scpc.println("Zeroing complete.");
        scesp.println("Zeroing complete.");
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

void checkSerialPC(){
  if(scpc.check()){
    if(scpc.contains("id")){ // get id
      scpc.println("BM");
    }
    else if(scpc.contains("0")){  // stop
      stepper.stop(HARD);
      if(motorstate == states::ZEROING){
        direc = direcOld;
        }
//      stepper.moveSteps(15000);
      scpc.println("Motor Stopped");
      motorstate = states::IDLE;
    }
    else if(scpc.contains("sv")){ // set speed
      if(motorstate != states::ZEROING){
        if(scpc.toInt16() >= 25 && scpc.toInt16() <= maxvelocity){
          velocity = scpc.toInt16(); 
          if(motorstate == states::MOVING){
            stepper.setMaxVelocity(velocity);
          }
          scpc.print("Speed has been set to: ");
          scpc.println(velocity);
        }
        else{
          scpc.println("Could not set speed to input value");
        }
      }
      else if(motorstate == states::ZEROING){
        scpc.println("Please do not change speed during zeroing!");
      }
    }
    else if(scpc.contains("vu")){ // set up (nudge up) the velocity
      if(motorstate != states::ZEROING){
        if(velocity >= maxvelocity){
          scpc.println("Max velocity reached");        
        }
        else{
          velocity += 25;
          if(motorstate == states::MOVING){
            stepper.setMaxVelocity(velocity);
          }
        }
      }
      else if(motorstate == states::ZEROING){
        scpc.println("Please do not change speed during zeroing!");
      }
    }
    else if(scpc.contains("vd")){ // set down (nudge down) the velocity
      if(motorstate != states::ZEROING){
        if(velocity <= 25){
          scpc.println("Min velocity reached");        
        }
        else{
          velocity += -25;
          if(motorstate == states::MOVING){
            stepper.setMaxVelocity(velocity);
          }
        }   
      }
      else if(motorstate == states::ZEROING){
        scpc.println("Please do not change speed during zeroing!");
      }
    }
    else if(scpc.contains("gm")){ // get mode 
      scpc.print("Bouncymotor status is currently: ");
      if(motorstate == states::MOVING){
        scpc.println("controlled motion");
      }
      else if(motorstate == states::ZEROING){
        scpc.println("zeroing");
      }
      else if(motorstate == states::IDLE){
        scpc.println("idle and awaiting instruction o7");
      }
    }
    else if(scpc.contains("gv")){ // get velocity 
      if(motorstate == states::ZEROING){
        scpc.println("Current velocity could not be obtained as Bouncymotor is zeroing.");
      }
      else{
        scpc.print("Current velocity is set to: ");
        scpc.println(velocity);
      }
    }
    else if(scpc.contains("gp")){ // get position
//      angle = stepper.encoder.getAngleMoved();
//      distance = angle*8/360;
//      scpc.print("Angle: ");
//      scpc.println(angle);
//      scpc.print("Distance: ");
//      scpc.println(distance);  
      scpc.print("Microstep: ");
      scpc.println(stepper.driver.getPosition());
    }
    else if(scpc.contains("ze")){ // zero the stage to top microswitch
      direcOld = direc;
      zeroing(true);
      motorstate = states::ZEROING;
      scpc.println("Zeroing...");
    }
    else if(scpc.contains("k")){  // toggle run/stop
//      upstop = digitalRead(7);
      if(motorstate == states::IDLE){
        //if topstop hit and td = up OR if botstop hit and td = down, print NO, else...
        if(upstop == 0 && direc == false){
          scpc.println("Please change direction before moving!");
        }
        else if(downstop == 0 && direc == true){
          scpc.println("Please change direction before moving!");
        }
        else{
          motorstate = states::MOVING;
          stepper.setMaxVelocity(velocity);
          stepper.runContinous(direc);
          //scpc.println(direc);
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
    else if(scpc.contains("td")){ // toggle direction
      direc = !direc;
      scpc.print("direction set to: ");
      if(direc == true){
        scpc.println("down"); 
      }
      else{
        scpc.println("up");
      }
      if(motorstate == states::MOVING){
        stepper.stop(HARD);
        stepper.runContinous(direc);
      }
    }
    else if(scpc.contains("su")){ // set direc to up
      if(direc == 1){
        direc = 0;
        scpc.println("direction set to: up");
        if(motorstate == states::MOVING){
          stepper.stop(HARD);
          stepper.runContinous(direc);
        }
      }
      else if(direc == 0){
        scpc.println("Already moving upward.");
      }
    }
    else if(scpc.contains("sd")){ // set direc to down
      if(direc == 0){
        direc = 1;
        scpc.println("direction set to: down");
        if(motorstate == states::MOVING){
          stepper.stop(HARD);
          stepper.runContinous(direc);
        }
      }
      else if(direc == 1){
        scpc.println("Already moving downward.");
      }
    }
    else if(scpc.contains("ms")){
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

void checkSerialESP(){
  if(scesp.check()){
    if     (scesp.contains("id")){ // get id
      scesp.println("BM");
    }
    else if(scesp.contains("0")){  // stop
      stepper.stop(HARD);
      if(motorstate == states::ZEROING){
        direc = direcOld;
        }
//      stepper.moveSteps(15000);
      scesp.println("Motor Stopped");
      motorstate = states::IDLE;
    }
    else if(scesp.contains("sv")){ // set speed
      if(motorstate != states::ZEROING){
        if(scesp.toInt16() >= 25 && scesp.toInt16() <= maxvelocity){
          velocity = scesp.toInt16(); 
          if(motorstate == states::MOVING){
            stepper.setMaxVelocity(velocity);
          }
          scesp.print("Speed has been set to: ");
          scesp.println(velocity);
        }
        else{
          scesp.println("Could not set speed to input value");
        }
      }
      else if(motorstate == states::ZEROING){
        scesp.println("Please do not change speed during zeroing!");
      }
    }
    else if(scesp.contains("vu")){ // set up (nudge up) the velocity
      if(motorstate != states::ZEROING){
        if(velocity >= maxvelocity){
          scesp.println("Max velocity reached");        
        }
        else{
          velocity += 25;
          if(motorstate == states::MOVING){
            stepper.setMaxVelocity(velocity);
          }
        }
      }
      else if(motorstate == states::ZEROING){
        scesp.println("Please do not change speed during zeroing!");
      }
    }
    else if(scesp.contains("vd")){ // set down (nudge down) the velocity
      if(motorstate != states::ZEROING){
        if(velocity <= 25){
          scesp.println("Min velocity reached");        
        }
        else{
          velocity += -25;
          if(motorstate == states::MOVING){
            stepper.setMaxVelocity(velocity);
          }
        }   
      }
      else if(motorstate == states::ZEROING){
        scesp.println("Please do not change speed during zeroing!");
      }
    }
    else if(scesp.contains("gm")){ // get mode 
      scesp.print("Bouncymotor status is currently: ");
      if(motorstate == states::MOVING){
        scesp.println("controlled motion");
      }
      else if(motorstate == states::ZEROING){
        scesp.println("zeroing");
      }
      else if(motorstate == states::IDLE){
        scesp.println("idle and awaiting instruction o7");
      }
    }
    else if(scesp.contains("gv")){ // get velocity 
      if(motorstate == states::ZEROING){
        scpc.println("Current velocity could not be obtained as Bouncymotor is zeroing.");
      }
      else{
        scesp.print("Current velocity is set to: ");
        scesp.println(velocity);
      }
    }
    else if(scesp.contains("gp")){ // get position
//      angle = stepper.encoder.getAngleMoved();
//      distance = angle*8/360;
//      scesp.print("Angle: ");
//      scesp.println(angle);
//      scesp.print("Distance: ");
//      scesp.println(distance);  
      scesp.print("Microstep: ");
      scesp.println(stepper.driver.getPosition());
    }
    else if(scesp.contains("ze")){ // zero the stage to top microswitch
      direcOld = direc;
      zeroing(true);
      motorstate = states::ZEROING;
      scesp.println("Zeroing...");
    }
    else if(scesp.contains("k")){  // toggle run/stop
//      upstop = digitalRead(7);
      if(motorstate == states::IDLE){
        //if topstop hit and td = up OR if botstop hit and td = down, print NO, else...
        if(upstop == 0 && direc == false){
          scesp.println("Please change direction before moving!");
        }
        else if(downstop == 0 && direc == true){
          scesp.println("Please change direction before moving!");
        }
        else{
          motorstate = states::MOVING;
          stepper.setMaxVelocity(velocity);
          stepper.runContinous(direc);
          //scesp.println(direc);
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
    else if(scesp.contains("td")){ // toggle direction
      direc = !direc;
      scesp.print("direction set to: ");
      if(direc == true){
        scesp.println("down"); 
      }
      else{
        scesp.println("up");
      }
      if(motorstate == states::MOVING){
        stepper.stop(HARD);
        stepper.runContinous(direc);
      }
    }
    else if(scesp.contains("su")){ // set direc to up
      if(direc == 1){
        direc = 0;
        scesp.println("direction set to: up");
        if(motorstate == states::MOVING){
          stepper.stop(HARD);
          stepper.runContinous(direc);
        }
      }
      else if(direc == 0){
        scesp.println("Already moving upward.");
      }
    }
    else if(scesp.contains("sd")){ // set direc to down
      if(direc == 0){
        direc = 1;
        scesp.println("direction set to: down");
        if(motorstate == states::MOVING){
          stepper.stop(HARD);
          stepper.runContinous(direc);
        }
      }
      else if(direc == 1){
        scesp.println("Already moving downward.");
      }
    }
    else if(scesp.contains("ms")){
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
