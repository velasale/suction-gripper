#include <Stepper.h>
#include <elapsedMillis.h>
#include <Keyboard.h>

#define Serial SerialUSB                      //This trick is meant for the Aduino Zero board
#define MotorInterfaceType 4
#define VALVE 7


const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper stepper(stepsPerRevolution, 9, 10, 11, 12);

/************** Stepper Motor Parameters ****************/
/*--- Enable pins ---*/
const byte enablePinA = 8;
const byte enablePinB = 13;
/*--- Travel steps ---*/
const int steps = 25;                         // 200steps = 1rev = 8mm,  25steps = 1/8rev = 1mm, 5steps = 0.2mm
const int distance = 60 * (200 / 8);          // 60mm * (1rev / 8mm * 200 steps / 1rev) = 1500 
const int clamp_distance = 15 * (200 / 8);    // 10mm * (1rev / 8mm * 200 steps / 1rev)
//const int clamp_distance = 30 * (200 / 8);  // 10mm * (1rev / 8mm * 200 steps / 1rev)
const int initial_distance = distance - clamp_distance;
int target;
/*--- Travel speed ---*/
/* If using the L298N driver, speed should be within these ranges:
/* AccelStepper speeds:  450 < x < 1100   */
/* Stepper speeds:       140 < x < 330    */
const int close_speed = 140;               // s/re = 2000 --
const int open_speed = 330; 




void setup() {

  Serial.begin(9600);
  Keyboard.begin(); 


  // Initialize VALVE pin as output
  delay(10);
  pinMode(VALVE, OUTPUT);
  delay(10);
  digitalWrite(VALVE, LOW);
  delay(10);

  // Initialize STEPMOTOR pins 
  pinMode(enablePinA, OUTPUT);
  pinMode(enablePinB, OUTPUT);
  delay(10); 

 
}



void loop() {
 
  char key;  
  key = Serial.read();

  /*********************** MOVE UP OR DOWN ******************/
  if (key == 'u'){
    Serial.println("");
    Serial.println("Moving up");
    motorSteps(open_speed, initial_distance);
    delay(100);  
    motorSteps(close_speed, clamp_distance);  
  }
  if (key == 'd'){
    Serial.println("");
    Serial.println("Moving down");    
    motorSteps(open_speed,-distance);         
  }


  /************** MOVE ONE STEP AT A TIME ******************/
  if (key == 'w'){
    Serial.println("");
    Serial.println("Moving one step up");    
    motorSteps(close_speed, steps);      
  }
  if (key == 's'){
    Serial.println("");
    Serial.println("Moving one step up");
    motorSteps(close_speed, - steps);  
  }
  

  /*********************** VALVE **************************/
  if (key == 'o'){
    digitalWrite(VALVE, HIGH);
    Serial.println("");
    Serial.println("Valve open");
  }
  if (key == 'l'){
    digitalWrite(VALVE, LOW);
    Serial.println("");
    Serial.println("Valve closed");
  }
  
}


void motorSteps(int stp_speed, int stp_distance){    
  digitalWrite(enablePinA, HIGH);
  digitalWrite(enablePinB, HIGH);     
   
  stepper.setSpeed(stp_speed);
  stepper.step(stp_distance);

  Serial.println("Finished moving");

  digitalWrite(enablePinA, LOW);
  digitalWrite(enablePinB, LOW);      
  delay(100);            
  
}
