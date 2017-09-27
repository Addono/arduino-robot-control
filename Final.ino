/*
 * @author      Adriaan Knapen <a.d.knapen@student.tue.nl> <adriaan.knapen@gmail.com>
 * @copyright   Copyright (c) 2016, Adriaan Knapen
 * @licence     http://opensource.org/licenses/MIT  MIT License
 * @description Controls a gimbal and two DC motors over a Serial1 connection with the keyboard.
 * @version     v0.3.1
 * 
 * The DC motors are controlled with the numpad, and the gimbal with WASD. Press 'c' 
 * to reset the camera to its default position. 
 */

#include "String.h"
#include <Servo.h>  // Import the servo library for the gimbal control.

Servo GimX, GimY, Net, motorA, motorB;

// The minimum, maximum, and initial position of the net.
int minNet = 0;
int maxNet = 160;
int initNet = 90;

// The stepsize of the net movement.
int netStep = 5;

// Stores the net position.
int NetPos = initNet;

// Sets the default center angles of the gimbal.
int GimPosXCent = 80;
int GimPosYCent = 5;

// Set the minimum and maximum angle of the gimbal
int minGimX = 0;
int maxGimX = 180;

int minGimY = 5;
int maxGimY = 180;

// Set the stepsize of the gimbal.
int GimStep = 5;

// Stores the position of the gimbal
int GimPosX = GimPosXCent;
int GimPosY = GimPosYCent;

// Set the output pins for the motor controller (L298N)
int motorMin = 0;
int motorMax = 180;
int motorStep = 5;
int initMotor = 90;

// Pins of the 
int GimXPin = A0;
int GimYPin = A1;

int NetPin = A2;

int motorAPin = A3;
int motorBPin = A4;

// Create a buffer to store the currently transmitted message.
char    charBuf = -1;

/**
 * Delays execution while letting a led blink.
 * @param long The amount of miliseconds the delay should last.
 * @author Unknown
 */
void WaitAndBlink( unsigned long DeltaMilliSec, int ledPin){
  // wait DeltaMilliSec milliseconds, LED blinks as a sign of life
  // as time passes frequency increases
  unsigned long DeltaT = 0;
  unsigned long TZero = millis(); //get start time
  while(DeltaT<DeltaMilliSec){
    unsigned long TCurrent = millis();
    DeltaT = TCurrent-TZero;      //compute elapsed time
    delay(500-400*DeltaT/DeltaMilliSec);
    digitalWrite(ledPin,LOW);
    delay(500-400*DeltaT/DeltaMilliSec);
    digitalWrite(ledPin,HIGH);
  } 

  // Turn the LED off when the delay is finished.
  digitalWrite(ledPin, LOW);
}  

/**
 * Changes the position of the net.
 * @param int The angle difference compared to the previous angle.
 */
void changeNetPos(int delta) {
    setNetPos(delta + NetPos);
}

/**
 * Changes the angle of the net.
 * @param int The angle of the net. 
 */
void setNetPos(int pos) {
    NetPos = getWithinBounds(pos, minNet, maxNet);
    
    Serial.print("Net position: ");
    Serial.println(NetPos);
    
    Net.write(NetPos);
}
 
/**
 * Changes the angle of the gimbal relative to its previous angle.
 * @param int The relative change of the x-angle.
 * @param int The relative change of the y-angle.
 */
void changeGimPos(int deltaX, int deltaY) {
  // Generate the relative x and y angle of the gimbal.
  int x = GimPosX + deltaX;
  int y = GimPosY + deltaY;

  // Change the angle of the gimbal servos to the new value.
  setGimPos(x, y);
}

/**
 * Sets the angles of the gimbal.
 * @param int The x-angle.
 * @param int the y-angle.
 */
void setGimPos(int x, int y) {
  // Make sure the gimbal positions are within bounds.
  GimPosX = getWithinBounds(x, minGimX, maxGimX);
  GimPosY = getWithinBounds(y, minGimY, maxGimY);

  // Print the new gimbal position to the command line.
  Serial.print("Position: ");
  Serial.print(GimPosX);
  Serial.print(" ");
  Serial.println(GimPosY);

  // Write the new gimbal angles to the servos.
  GimX.write(GimPosX);
  GimY.write(GimPosY);
}

/**
   Changes the velocity relative to the current velocity.
   @param int Velocity differenct of motor A.
   @param int Velocity differenct of motor B.
*/
void alterVelocity(int deltaA, int deltaB) {
  int newA = motorA.read() + deltaA;
  int newB = motorB.read() + deltaB;

  setVelocity(newA, newB);
}

/**
   Sets the velocity of both motors.
   @param int The speed of motor A.
   @param int The speed of motor B.
*/
int setVelocity(int newVelA, int newVelB) {
  // Check if the new motor velocity values are within bounds, else snap them to maximal.
  int velA = getWithinBounds(newVelA, motorMin, motorMax);
  int velB = getWithinBounds(newVelB, motorMin, motorMax);

  Serial.print("New velocity: ");
  Serial.print(velA);
  Serial.print(" ");
  Serial.println(velB);

  // Write the new speed to the motors.
  motorA.write(velA);
  motorB.write(velB);
}

/**
   Checks if the given velocity is within bounds, else snaps them to the maximals.
   @param int The velocity which has to be checked.
   @return int The velocity within bounds.
*/
int getWithinBounds(int value, int lower, int upper) {
  if (value > upper) {
    return upper;
  } else if (value < lower) {
    return lower;
  } else {
    return value;
  }
}

/**
   Processes the buffer.
*/
void processBuffer(char thisBuf) {
  Serial.print(thisBuf);
  Serial.print("  -  ");

  // Set movement with incrementing stepsize.
  switch (thisBuf) {
    // Stop all
    case '5':
      Serial.println("Movement stopped");
      setVelocity(initMotor, initMotor);
      break;

    // Stop left
    case '4':
      Serial.println("Stopped right motor");
      setVelocity(motorA.read(), initMotor);
      break;

    // Stop all
    case '6':
      Serial.println("Stopped left motor");
      setVelocity(initMotor, motorB.read());
      break;

    // Move forward
    case '8':
      Serial.println("Increasing both motors");
      alterVelocity(motorStep, motorStep);
      break;

    // Move backwards.
    case '2':
      Serial.println("Decreasing both motors");
      alterVelocity(-motorStep, -motorStep);
      break;

    // Move forwards left.
    case '7':
      Serial.println("Increasing left motor");
      alterVelocity(0, motorStep);
      break;

    // Move forwards right.
    case '9':
      Serial.println("Increasing right motor");
      alterVelocity(motorStep, 0);
      break;

    // Move backwards left.
    case '1':
      Serial.println("Decreasing left motor");
      alterVelocity(0, -motorStep);
      break;

    // Move backwards right.
    case '3':
      Serial.println("Decreasing right motor");
      alterVelocity(-motorStep, 0);
      break;

    case '/':
      setVelocity(motorMax, motorMax);
      Serial.println("Both maximum speed forwards");
      break;

    case 'w':
      changeGimPos(0, GimStep);
      break;

    case 's':
      changeGimPos(0, -GimStep);
      break;

    case 'a':
      changeGimPos(GimStep, 0);
      break;

    case 'd':
      changeGimPos(-GimStep, 0);
      break;
      
    case '*':
      changeNetPos(netStep);
      break;
      
    case '-':
      changeNetPos(-netStep);
      break;

    case 'c':
      setGimPos(GimPosXCent, GimPosYCent);
      break;

    default:
      Serial.println("Command not found");
      break;
  }
}

void setup() {
  // Attach the servos.
  GimX.attach(GimXPin);
  GimY.attach(GimYPin);
  Net.attach(NetPin);
  motorA.attach(motorAPin);
  motorB.attach(motorBPin);

  // Set the servos to their inital position.
  GimX.write(GimPosX);
  GimY.write(GimPosY);
  Net.write(NetPos);
  motorA.write(initMotor);
  motorB.write(initMotor);
  
  Serial.begin(9600); // in case of USB connection
  Serial1.begin(115200); // in case of ethernet cable or WiFi

  WaitAndBlink(60000, 13);

  Serial.println("Welcome, booting complete. We are ready to dominate the world.\n");
}

void loop() {
  // Check if something new can be read from the network serial connection.
  if(Serial1.available() > 0)
  {
    processBuffer((char) Serial1.read());
  }

  // Check if something new can be read from the USB serial connection.
  if(Serial.available() > 0) {
    processBuffer((char) Serial.read());
  }
}
