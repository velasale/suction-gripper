
/*
 Stepper Motor Control - one revolution

 This program drives a unipolar or bipolar stepper motor.
 The motor is attached to digital pins 8 - 11 of the Arduino.

 The motor should revolve one revolution in one direction, then
 one revolution in the other direction.


 Created 11 Mar. 2007
 Modified 30 Nov. 2009
 by Tom Igoe

 */
 
#define Serial SerialUSB    //This trick is meant for the Aduino Zero board


#include <Stepper.h>

const byte enablePinA = 12;
const byte enablePinB = 13;

const int stepsPerRevolution = 400;  // change this to fit the number of steps per revolution
const int steps = 1350;
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

void setup() {
  // set the speed at 60 rpm:
  myStepper.setSpeed(100);
  // initialize the serial port:
  Serial.begin(9600);
}

void loop() {
  // step one revolution  in one direction:
  Serial.println("clockwise");
  digitalWrite(enablePinA, HIGH);
  digitalWrite(enablePinB, HIGH);  
  myStepper.step(steps);

  // With a screw-nut driver, there is no need to leave the driver on
  digitalWrite(enablePinA, LOW);
  digitalWrite(enablePinB, LOW);  
  delay(1500);

  // step one revolution in the other direction:
  Serial.println("counterclockwise");
  digitalWrite(enablePinA, HIGH);
  digitalWrite(enablePinB, HIGH);  
  myStepper.step(-steps);

  // With a screw-nut driver, there is no need to leave the driver on
  digitalWrite(enablePinA, LOW);
  digitalWrite(enablePinB, LOW);  
  delay(1500);
}
