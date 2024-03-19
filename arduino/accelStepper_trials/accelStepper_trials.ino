#include <AccelStepper.h>
#include <elapsedMillis.h>
#include <Keyboard.h>

#define Serial SerialUSB                      //This trick is meant for the Aduino Zero board
#define MotorInterfaceType 4
#define VALVE 7

AccelStepper stepper = AccelStepper(MotorInterfaceType, 9, 10, 11, 12);
elapsedMillis printTime;


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
// For driver L298N: min_speed = 450    max_speed = 1100
const int close_speed = 450;               
const int open_speed = 1100; 



void setup() {

  Serial.begin(9600);
  Keyboard.begin(); 

  stepper.setMaxSpeed(1200);                  // steps/second
  stepper.setAcceleration(2000);

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
  
  target = stepper.currentPosition() + stp_distance;     
  stepper.moveTo(target);   
  while (stepper.currentPosition() != target){          
      stepper.setSpeed(stp_speed);    // Need to go in the same loop as runSpeedToPosition          
      stepper.runSpeedToPosition();   //doesnt implement acceleration             
  }

  Serial.println("Finished moving");

  digitalWrite(enablePinA, LOW);
  digitalWrite(enablePinB, LOW);                  
  
}
