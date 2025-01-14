#include <Arduino.h>
#include <BlueMotor.h>
#include <Romi32U4.h>
#include "Timer.h"


long oldValue = 0;
long newValue;
long volatile count = 0;
unsigned time = 0;
const unsigned ENCA = 0;
const unsigned ENCB = 1;
float Kp = 1; 

BlueMotor::BlueMotor()
{
}

void BlueMotor::setup()
{
    pinMode(PWMOutPin, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(ENCA, INPUT);
    pinMode(ENCB, INPUT);
    TCCR1A = 0xA8; //0b10101000; //gcl: added OCR1C for adding a third PWM on pin 11
    TCCR1B = 0x11; //0b00010001;
    ICR1 = 400;
    OCR1C = 0;
    attachInterrupt(digitalPinToInterrupt(ENCA), isrA, CHANGE); //initializes encoder A
    attachInterrupt(digitalPinToInterrupt(ENCB), isrB, CHANGE); //initializes encoder B
    reset();
}


long BlueMotor::getPosition()
{
    long tempCount = 0;
    noInterrupts();
    tempCount = count;
    interrupts();
    return tempCount;
}

void BlueMotor::reset()
{
    noInterrupts();
    count = 0;
    interrupts();
}

void BlueMotor::isrA()
{
  if (digitalRead(ENCA) == digitalRead(ENCB)) { //uses on of two encoders
    count++;
  }
  else {
    count--;
  }
}

void BlueMotor::isrB() //uses the other of two encoders
{
  if (digitalRead(ENCA) == digitalRead(ENCB)) {
    count--;
  }
  else {
    count++;
  }
}

void BlueMotor::readENCS() //function to print out the value of the encoders, to test if encoder is functioning within blue motor, if needed
{
  Serial.print(digitalRead(ENCA));
  Serial.print("     "); 
  Serial.println(digitalRead(ENCB)); 
}

void BlueMotor::setEffort(int effort)
{
    if (effort < 0)
    {
        setEffort(-effort, true);
    }
    else
    {
        setEffort(effort, false);
    }
}

void BlueMotor::setEffort(int effort, bool clockwise)
{
    if (clockwise)
    {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    }
    else
    {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    }
    OCR1C = constrain(effort, 0, 400);
}

void BlueMotor::moveTo(long target)  //USED FOR PART 2. Move to this encoder position within the specified
{                                    //tolerance in the header file using proportional control
                                     //then stop                                
    setEffort(Kp*(target - count));
    delay(30); 
    if (abs(target-count) <= 5)  { setEffort(0); }  

}

void BlueMotor::moveToWithoutDB(double DBcorrection, long target, double dir)  //USED FOR PART 3b. passes the function to the DBcorrection function that will correct the dead band
{
  setEffortWithoutDB(DBcorrection, target, dir);
}

void BlueMotor::setEffortWithoutDB(double DBcorrection, double effort, double dir) //USED FOR PART 3b. function that corrects the dead band
{
    double adjEffort = DBcorrection + ((effort/400.0)*(400-DBcorrection)); //calculation to correct dead band
    double dirAdjEffort = dir*adjEffort; //direction taken into account
    if (!(abs(dirAdjEffort - Kp*count) <= 5)) { //will run the moveTo() function so long as count doesn't reach the target up to a tolerance value
    moveTo(dirAdjEffort); 
    }
  }
    

void BlueMotor::deadBand(int Dir) //USED FOR PART 3a. function to identify the value at which dead band is removed, the minimum effort value to make the motor move with the fourbar
{
    int speedIncrements = 1; 
    for (int i = 0; i <= 1200; i++) {
    int effort = Dir*speedIncrements*i; 
    setEffort(effort); 
    Serial.print(effort); 
    Serial.print("     ");
    Serial.println(count); 
    delay(10); 
    }
    setEffort(0); 
}

long volatile BlueMotor::releaseCount() { //function that can get the count value to the main class, if needed
  return count; 
}
