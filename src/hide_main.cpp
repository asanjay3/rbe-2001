// #include <Arduino.h>
// #include <Romi32U4.h>
// #include "BlueMotor.h"
// #include <servo32u4.h>
// #include "Timer.h"



// BlueMotor blueMotor;
// Servo32U4 servoMotor; 
// Romi32U4ButtonB buttonB;
// Romi32U4ButtonC buttonC;

// int potPin = A0; //potentiometer pin reading
// int servoPin = 5; //servomotor power pin
// int servoSensePin = A0; //servomotor sensing pin
// int jawOpen = 45; //value at which linear gripper is most open
// int jawClosed = 975; //value at whihc linear gripper closes a plate
// int potPinTracker = analogRead(potPin); //to track value of potentiometer
// int potPinTracker2 = 0; 
// int timeTracker = millis() + 100;  //time interval
// void linearGripper(); 
// void bottomOutGripper();
// void openBOG(int DBcorrection, int effort, int dir); 
// void closeBOG(int DBcorrection, int effort, int dir); 
// long newPosition = 0;
// long oldPosition = 0; 
// int now = 0; 
// int CPR = 270; //counts per revolution
// int sampleTime = 100; 
// int bottomTracker = 0;
// int bottomTracker2 = 1;
// unsigned t = 1500;  

// void setup()
// {
//   Serial.begin(9600);
//   blueMotor.setup(); //sets up blue motor
//   blueMotor.reset();
//   delay(3000); 
//   servoMotor.attach(); //sets up servomotor
//   delay(3000);
//   servoMotor.setMinMaxMicroseconds(500, 2500); //establishes range of servo motor
// }


// void loop() //will call whatever function defined below based on lab activity number
// {
//  }

// void deadBandDataCollection() { //USED FOR PART 3b. function to get time, user input effort, applied effort, and speed in RPM
//   int timeToPrint = millis() + sampleTime;
//   oldPosition = blueMotor.getPosition();
//   int DBcorrection = 215; //added buffer included as a precaution so that motor moves just in case gears stuck or misaligned while moving
//   //int DBcorrection = 185;   //DB correction uses 215 on the way up and 185 on the way down
//   int dir = -1;
//   int initTime = millis(); //establishes the initial time count at which operation starts

//    for (int i = 0; i <= 400; i++) { //increments effort until 400
//       blueMotor.setEffortWithoutDB(DBcorrection, i, dir); //calls function in blue motor class
//       while(!((now = millis()) > timeToPrint));
//       if ((now = millis()) > timeToPrint) {
//         timeToPrint = now + sampleTime;
//         newPosition = blueMotor.getPosition(); 
//         int speedInRPM = (((newPosition - oldPosition)*60000) / (sampleTime * CPR)); //speed in rpm calculation
//         float appliedEffort = (dir*(DBcorrection + ((i/400.0)*(400-DBcorrection)))); //applied effort calculation
//         Serial.print(timeToPrint - initTime); //subtracts current time from initial time
//         Serial.print("     "); 
//         Serial.print(i); 
//         Serial.print("     "); 
//         Serial.print(appliedEffort);
//         Serial.print("          ");
//         Serial.println(speedInRPM);
//         oldPosition = newPosition;
//         }
//         }
//         blueMotor.setEffort(0); 
// }


// void openBOG(int DBcorrection, int effort, int dir) { //USED FOR PART 5. function to raise arm and deposit plate
//   blueMotor.moveToWithoutDB(DBcorrection, effort, dir); //raises fourbar
//   unsigned t = 1400; 
//     while (t >= 500) { //closes gripper mouth
//     servoMotor.writeMicroseconds(t);
//     delay(30); 
//     t-= 10.0; 
//     }
//     }

// void closeBOG(int DBcorrection, int effort, int dir) { //USED FOR PART 5. function to raise the arm and collect gripper

//   blueMotor.moveToWithoutDB(DBcorrection, effort, dir); //raises fourbar  
//   if (t >= 500) {  //opens gripper mouth
//     servoMotor.writeMicroseconds(t);
//     delay(100); 
//     t-= 10.0; 
//     }
//   //while (t < 500) { blueMotor.moveToWithoutDB(DBcorrection, effort, -dir); }
// }

// void bottomOutGripper() { //USED FOR PART 4. function to test bottom out gripper function. closes on a plate and then goes down. if stuck before reach to one plate close position, goes back down
//   servoMotor.attach();
//   unsigned t = 500; 
//   servoMotor.writeMicroseconds(500); 
//   timeTracker = millis() + 100;
//     while (t < 1400) { //closes gripper mouth
//     servoMotor.writeMicroseconds(t);
//     delay(30); 
//     t+= 10.0; 
//     if((now = millis()) > timeTracker)
//     {
//       timeTracker = now + 100;
//       bottomTracker2 = analogRead(servoSensePin); 
//       Serial.print(bottomTracker2); 
//       Serial.print("    ");
//       Serial.println(bottomTracker); 
//       if (bottomTracker2 == bottomTracker) { //senses if stuck, then goes down 
//         while (t > 500) {
//           t -= 10; 
//           servoMotor.writeMicroseconds(t); 
//         }
//       }
//       bottomTracker = bottomTracker2;  
//     } 
// }
// servoMotor.detach(); 
// }



// void linearGripper() { //USED FOR PART 4. function to test linear gripper function. closes gripper on one plate. if stuck before reaching one plate, goes back down
//   while ((analogRead(potPin) < jawClosed)) { //closes gripper while has not reached one plate position
//     servoMotor.writeMicroseconds(-35); 
//     if((now = millis()) > timeTracker)
//     {
//       timeTracker = now + 100; 
//       potPinTracker2 = analogRead(potPin); 
//       if (potPinTracker2 == potPinTracker) {  //if stuck, opens it
//         while (analogRead(potPin) > jawOpen) {
//          servoMotor.writeMicroseconds(35); 
//         }
//       }
//       potPinTracker = analogRead(potPin); 
//     }  
//     }
// }



    


  

  
