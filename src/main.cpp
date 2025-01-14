#include <Arduino.h>
#include <Romi32U4.h>
#include <servo32u4.h>
#include "Timer.h"
#include <Chassis.h>
#include "RemoteConstants.h"
#include <IRdecoder.h>
#include <Rangefinder.h>
#include <BlueMotor.h>

int irRemotePin = 14;

enum stateChoices{RAISETOROOF, CLOSEBOG, TURN, LINEFOLLOW, OPENBOG, HANDOFF, MIDFIELD, REVERSE, SPECIALTURN, TURN90, FINALAPPROACH, STOP};   
stateChoices state;
stateChoices nextSTATE; 
BlueMotor blueMotor;
Servo32U4 servoMotor; 
Romi32U4ButtonB buttonB;
Chassis chassis; 
Rangefinder rangefinder(17, 12); 
IRDecoder decoder(irRemotePin); 

int potPin = A0;
int servoPin = 5; 
int servoSensePin = A0; 
int jawOpen = 45;
int jawClosed = 975;
int potPinTracker = analogRead(potPin); 
int potPinTracker2 = 0; 
int timeTracker = millis() + 100;  
void linearGripper(); 
void bottomOutGripper();
void openBOG(); 
void closeBOG();  
void openLG(); 
void closeLG(); 
void firstRobotMachine(); 
void secondRobotMachine(); 
void lineFollow(); 
void handleKeyPress(int keyPress); 
int now = 0; 
int sampleTime = 100; 
int bottomTracker = 0;
int bottomTracker2 = 1; 
int leftReflectance = 20;
int rightReflectance = 22; 
bool bogRobot = true;  //ROBOT ONE IS BOTTOM OUT GRIPPER, ROBOT TWO IS LINEAR GRIPPER
bool firstRobot = true; //the non-lazy robot is the starting robot (and does most of the work)
bool twentyFiveDegreeRoof = true; 
unsigned t = 500;  
int DBcorrection = 235; 
int effort = 500;  //TBD??
int dir = 1;
int turnDir = 1; 
bool offLine = false; 
bool collectedFromPlatform = false; 
int bmCountTracker;
bool enteredHandoff = false; 
bool handoffAligned = false; 
int lineFollowKP = .85; 
int lineFollowIteration = 0; 
int rangefinderDist = 0; 
int variable = 0; 
bool approachingMidfield = false; 
bool turnedAround = false; 
bool didSpecialTurn = false; 

void setup()
{
  Serial.begin(9600);
  blueMotor.setup();
  decoder.init(); 
  rangefinder.init(); 
  blueMotor.reset();
  state = RAISETOROOF;
  delay(3000); 
  servoMotor.attach();
  delay(3000);
  servoMotor.setMinMaxMicroseconds(10, 1600); 
  chassis.init(); 
  servoMotor.writeMicroseconds(1400); 
  servoMotor.detach();
  chassis.getLeftEncoderCount(true); 
  pinMode(irRemotePin, INPUT); 
}


void loop()
{
  //Serial.println("Start");
  uint8_t keyPress = decoder.getKeyCode(); 
  if (keyPress != -1) { handleKeyPress(keyPress); }
  // Serial.println(digitalRead(irRemotePin));
  //Serial.print(keyPress);
  //Serial.print("###");
  //Serial.println(digitalRead(irRemotePin)); 
 //blueMotor.setEffort(-400); 

  Serial.print(rangefinder.getDistance()); 
  Serial.print("    ");
  Serial.println(state);
  switch(state)
  {
    case RAISETOROOF:
      blueMotor.moveToWithoutDB(DBcorrection, effort*25, dir); 
      bmCountTracker = blueMotor.releaseCount(); 
      // Serial.print((dir*(DBcorrection + ((effort/400.0)*(400-DBcorrection))))); 
      // Serial.print("      "); 
      // Serial.print(bmCountTracker); 
      // Serial.print("    ");
      // Serial.println((abs((dir*(DBcorrection + ((effort/400.0)*(400-DBcorrection)))) - bmCountTracker/12)));
      if (((abs((dir*(DBcorrection + ((effort/400.0)*(400-DBcorrection)))) - bmCountTracker/10) <= 50)) && (dir == 1) && (!collectedFromPlatform) && (!approachingMidfield)) //&& dir = 1
      {
        blueMotor.setEffort(0); 
        //servoMotor.writeMicroseconds(500); 
        chassis.driveFor(-10, -5, true); 
        //servoMotor.attach(); 
        state = CLOSEBOG; 
        //Serial.println("raising");
      }
      else if (((abs((dir*(DBcorrection + (((effort+45)/400.0)*(400-DBcorrection)))) - bmCountTracker/10) <= 2)) && (dir == 1) && (collectedFromPlatform) && (!approachingMidfield))
      {
        blueMotor.setEffort(0);
        chassis.driveFor(-8, -10, false); 
        nextSTATE = OPENBOG;  
        state = STOP; 
      }
      else if (((abs((dir*(DBcorrection + ((effort/400.0)*(400-DBcorrection)))) - bmCountTracker/10) <= 50)) && (dir == -1) && (!approachingMidfield))
      {
        blueMotor.setEffort(0);
        chassis.driveFor(10, 5, false);
        dir = 1; 
        state = TURN; 
      } 
      else if (((abs((dir*(DBcorrection + ((effort/400.0)*(400-DBcorrection)))) - bmCountTracker/10) <= 50)) && (dir == -1) && (approachingMidfield))
      {
        blueMotor.setEffort(0);
        chassis.setMotorEfforts(0, 0); 
        state = LINEFOLLOW; 
      }
      break; 

    case CLOSEBOG:
      //Serial.println(t); 
      if (t > 60)
      {
        servoMotor.writeMicroseconds(t); 
        delay(30); 
        t-= 10.0;
        //Serial.println(t);  
      }
      else if ((t <= 60) && (!collectedFromPlatform) && (!didSpecialTurn))
      { 
        chassis.driveFor(7, 20, false);
        dir = -1; 
        blueMotor.reset(); 
        nextSTATE = RAISETOROOF; 
        state = STOP; 
      }
      else if ((t <= 60) && (collectedFromPlatform) && (!didSpecialTurn) && (!enteredHandoff))
      {
        if (twentyFiveDegreeRoof) { turnDir = 1; } 
        else if (!twentyFiveDegreeRoof) { turnDir = -1; }
        nextSTATE = TURN;  
        state = STOP; 
      }
      else if ((t <= 60) && (enteredHandoff) && (!didSpecialTurn))
      {
        nextSTATE = REVERSE; 
        state = STOP; 
      }
      else if ((t <= 60) && (didSpecialTurn))
      {
        handoffAligned = false; 
        nextSTATE = FINALAPPROACH; 
        state = STOP; 
      }
     // servoMotor.detach(); 
      break; 

    case TURN: 
      if (!offLine)
      {
        chassis.turnFor(-30*turnDir, 35, true);
        offLine = true;
      }
      else 
      {
        chassis.turnFor(-5*turnDir, 50, true); 
      }
      if (analogRead(rightReflectance) > 650)
      {
        offLine = false; 
        //turnDir = 1;
        state = LINEFOLLOW; 
      }
      break; 

    case LINEFOLLOW:
      chassis.setTwist(-10, -(((analogRead(rightReflectance)) - analogRead(leftReflectance))*0.2));
      rangefinderDist = rangefinder.getDistance(); 
      delay(1);
      rangefinderDist = rangefinder.getDistance(); 
      if ((analogRead(leftReflectance) >= 700 && analogRead(rightReflectance) >= 700) && (!approachingMidfield))
      {
        chassis.driveFor(-7.5, -20, true); 
        if ((twentyFiveDegreeRoof) && (!collectedFromPlatform)) { turnDir = -1; }
        //else if ((twentyFiveDegreeRoof) && (enteredHandoff)) { turnDir = -1; }
        lineFollowIteration += 1; 
        state = TURN;
      }
      
      else if ((rangefinderDist < 7) && (!collectedFromPlatform) && (lineFollowIteration == 1))
      {
        blueMotor.reset(); 
        chassis.setMotorEfforts(0, 0); 
        state = OPENBOG; 
      }
      
      else if ((rangefinderDist < 20) && (collectedFromPlatform) && (!enteredHandoff))
      {
        dir = 1; 
        blueMotor.reset();
        chassis.setMotorEfforts(0, 0);
        state = RAISETOROOF; 
      }
      
      else if ((rangefinderDist < 3) && (collectedFromPlatform) && (enteredHandoff))
      {
        state = MIDFIELD; 
      }

      else if ((analogRead(leftReflectance) >= 600 && analogRead(rightReflectance) >= 600) && (approachingMidfield))
      {
        chassis.setMotorEfforts(0, 0);
        state = HANDOFF;
      }
      
      break;

    case OPENBOG:
    //collectedFromPlatform = true; 
    if (t < 1450)
    {
      servoMotor.writeMicroseconds(t); 
      delay(30); 
      t+= 10.0;
    }
    else if (t >= 1450)
    {
      dir = -1; 
      blueMotor.reset(); 
      if ((!collectedFromPlatform) && (!didSpecialTurn))
      {
        nextSTATE = CLOSEBOG; 
        collectedFromPlatform = true; 
      }
      else if ((collectedFromPlatform) && (!didSpecialTurn))
      {
        chassis.driveFor(7, 20, false); 
        nextSTATE = HANDOFF; 
      }
      else if (didSpecialTurn)
      {
        nextSTATE = CLOSEBOG;
      }
      state = STOP; 
    }
      break; 

    case HANDOFF:
      if (!enteredHandoff) 
      {
        chassis.setMotorEfforts(0, 0);
        blueMotor.setEffort(0);
        Serial.println("Stage: Handoff");
        if (twentyFiveDegreeRoof) { turnDir = -1; }
        else if (!twentyFiveDegreeRoof) { turnDir = 1; }
        enteredHandoff = true; 
        state = TURN; 
      }
      else if (enteredHandoff)
      {
        Serial.println("Handoff Again"); 
        nextSTATE = CLOSEBOG; 
        state = STOP; 
      }
      break; 

    case MIDFIELD:
      approachingMidfield = true;  
      if (!handoffAligned) 
      {
        state = TURN90; 
        break; 
      }
      dir = -1; 
      state = RAISETOROOF; 
      if (analogRead(leftReflectance) >= 700 && analogRead(rightReflectance) >= 700 && (didSpecialTurn))
      {
        state = OPENBOG; 
      } 
      break;   

    case REVERSE:
      if (!turnedAround) 
      {
        chassis.turnFor(-180, 50, true);    
        turnedAround = true; 
      }
      else if (turnedAround)
      {
        chassis.setTwist(-10, -(((analogRead(rightReflectance)) - analogRead(leftReflectance))*0.2)); 
        if (analogRead(leftReflectance) >= 600 && analogRead(rightReflectance) >= 600) 
        {
          chassis.driveFor(-5.5, -20, true); 
          state = SPECIALTURN; 
        }
      }
      break; 

    case SPECIALTURN:
      didSpecialTurn = true; 
      if (twentyFiveDegreeRoof) { turnDir = 1; }
      else if (!twentyFiveDegreeRoof) { turnDir = -1; }
      chassis.turnFor(turnDir*5, 50, true); 
      if (analogRead(rightReflectance) >= 723) 
      {
        nextSTATE = OPENBOG; 
        state = STOP; 
      }
      break; 

    case TURN90:
      handoffAligned = true; 
      chassis.turnFor(-turnDir*90, 20, true); 
      chassis.driveFor(-2, -20, true); 
      state = MIDFIELD;
      break; 

    case FINALAPPROACH:
      if (!handoffAligned)
      {
        chassis.turnFor(turnDir*90, 20, true); 
        handoffAligned = true;
      }
      else if (handoffAligned)
      {
        chassis.setTwist(-10, -(((analogRead(rightReflectance)) - analogRead(leftReflectance))*0.2));
        if (analogRead(leftReflectance) >= 600 && analogRead(rightReflectance) >= 600) 
        {
          chassis.setMotorEfforts(0, 0);
          nextSTATE = OPENBOG;
          state = STOP; 
        }
      }

    case STOP:
      //chassis.setWheelSpeeds(0, 0);
      if (keyPress == remoteUp) { state = nextSTATE; }
      break; 
    
    default:
      break; 
  }
 
}


void handleKeyPress(int keyPress) 
{
  if (keyPress == remotePlayPause) //need constants.h file
  {
    nextSTATE = state;
    state = STOP; 
  }

  if (keyPress == remoteUp) //need constants.h file
  {
    state = nextSTATE;
  }
}


void lineFollow() { //GET FROM PRANAV

}

void openBOG() {
  //blueMotor.moveToWithoutDB(DBcorrection, effort, dir); //raises fourbar
  unsigned t = 500; 
    while (t <= 1400) { //closes gripper mouth
    servoMotor.writeMicroseconds(t);
    delay(30); 
    t+= 10.0; 
    }
}

void openLG() {
  while ((analogRead(potPin) > jawOpen)) {
  servoMotor.writeMicroseconds(35); 
  }
}

void closeBOG(int DBcorrection, int effort, int dir) {
  blueMotor.moveToWithoutDB(DBcorrection, effort, dir); //raises fourbar  
  while (t < 1500) {  //opens gripper mouth
    servoMotor.writeMicroseconds(t);
    delay(100); 
    t+= 10.0; 
    }
  //while (t < 500) { blueMotor.moveToWithoutDB(DBcorrection, effort, -dir); }  
}

void closeLG() 
{
  while ((analogRead(potPin) < jawClosed)) {
    servoMotor.writeMicroseconds(-35);
    if((now = millis()) > timeTracker)
    {
      timeTracker = now + 100; 
      potPinTracker2 = analogRead(potPin); 
      if (potPinTracker2 == potPinTracker) {
        while (analogRead(potPin) > jawOpen) {
         servoMotor.writeMicroseconds(35); 
        }
      }
      potPinTracker = analogRead(potPin); 
    }  
    }
}

void bottomOutGripper() {
  servoMotor.attach();
  unsigned t = 500; 
  //servoMotor.writeMicroseconds(500); 
  timeTracker = millis() + 100;
    while (t < 1400) {
    servoMotor.writeMicroseconds(t);
    delay(50); 
    t+= 10.0; 
    // if((now = millis()) > timeTracker)
    // {
    //   timeTracker = now + 100;
    //   bottomTracker2 = analogRead(servoSensePin); 
    //   Serial.print(bottomTracker2); 
    //   Serial.print("    ");
    //   Serial.println(bottomTracker); 
    //   if (bottomTracker2 == bottomTracker) {
    //     while (t > 500) {
    //       t -= 10; 
    //       servoMotor.writeMicroseconds(t); 
    //     }
    //   }
    //   bottomTracker = bottomTracker2;  
    } 

// while (t >= 500) {
//   servoMotor.writeMicroseconds(t);
//     delay(30); 
//     t-= 10.0; 
//     Serial.println(t); 
// }
//servoMotor.detach(); 
}

void linearGripper() {
  while ((analogRead(potPin) < jawClosed)) {// && (potPinTracker != potPinTracker2)) {
    servoMotor.writeMicroseconds(-35);
    if((now = millis()) > timeTracker)
    {
      timeTracker = now + 100; 
      potPinTracker2 = analogRead(potPin); 
      if (potPinTracker2 == potPinTracker) {
        while (analogRead(potPin) > jawOpen) {
         servoMotor.writeMicroseconds(35); 
        }
      }
      potPinTracker = analogRead(potPin); 
    }  
    }
    while ((analogRead(potPin) > jawClosed)) {
      servoMotor.writeMicroseconds(35); 
    }
}



    


  

  
