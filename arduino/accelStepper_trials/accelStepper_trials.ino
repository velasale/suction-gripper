#include <AccelStepper.h>
#include <elapsedMillis.h>
#include <Keyboard.h>

#define Serial SerialUSB    //This trick is meant for the Aduino Zero board
#define MotorInterfaceType 4
#define VALVE 7

// The motor resolution is 200steps per revolution. 
// By double checking with Accelstepper, 800steps lead to 16mmm which is 2rev --> 400steps/rev

AccelStepper stepper = AccelStepper(MotorInterfaceType, 9, 10, 11, 12);


const byte enablePinA = 8;
const byte enablePinB = 13;

/******* Stepper Motor Parameters ****************/
const int steps = 25;                 // 200steps = 1rev = 8mm,  25steps = 1/8rev = 1mm, 5steps = 0.2mm
const int distance = 59 * (200 / 8);  // 60mm * (1rev / 8mm * 200 steps / 1rev) = 1500 
const int clamp_distance = 20 * (200 / 8); //10mm * (1rev / 8mm * 200 steps / 1rev) = 250
const int initial_distance = distance - clamp_distance;
/***** Travel speed *****/
const int close_speed = 700;    // speed1=450
const int open_speed = 1000; 


elapsedMillis printTime;



void setup() {

  Serial.begin(9600);
  Keyboard.begin();
  
  // put your setup code here, to run once:

  stepper.setMaxSpeed(1200);       // steps/second
  stepper.setAcceleration(1000);

  // Initialize VALVE pin as output
  delay(10);
  pinMode(VALVE, OUTPUT);
  delay(10);
  digitalWrite(VALVE, LOW);
  delay(10);
  
}



void loop() {
 
  char key;
  int target;
  

  key = Serial.read();

  if (key == 'u'){
    Serial.println("");
    Serial.println("Moving up");
    motorSteps2(open_speed, initial_distance);
    int target;     
    delay(100);  
    motorSteps2(close_speed, clamp_distance);  
  }


  if (key == 'd'){
    Serial.println("");
    Serial.println("Moving down");    
    motorSteps2(open_speed,-distance);         
  }


  /************** MOVE ONE STEP AT A TIME ***************/

  if (key == 'w'){
    Serial.println("");
    Serial.println("Moving one step up");    
    motorSteps2(close_speed, steps);      
  }

  if (key == 's'){
    Serial.println("");
    Serial.println("Moving one step up");
    motorSteps2(close_speed, - steps);  
  }
  

  /************** VALVE ****************/
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


void motorSteps(int g_speed, int g_accel, int g_pos){
  // Works
  
  digitalWrite(enablePinA, HIGH);
  digitalWrite(enablePinB, HIGH);  

  stepper.setMaxSpeed(g_speed);       // steps/second
  stepper.setAcceleration(g_accel);
  
  delay(10);  
  
  stepper.runToNewPosition(g_pos);    // WARNING this is a BLOCKING FUNCTION
  
    Serial.print(" ");
    Serial.println(stepper.currentPosition());
  
  digitalWrite(enablePinA, LOW);
  digitalWrite(enablePinB, LOW);  
  delay(1000);
  
}


void motorSteps2(int stp_speed, int stp_distance){
  // Works
  int target;
  int flag = 0;
  
  digitalWrite(enablePinA, HIGH);
  digitalWrite(enablePinB, HIGH);      
  target = stepper.currentPosition() + stp_distance;
  
  while (flag == 0){    
      stepper.setSpeed(stp_speed);  
      stepper.moveTo(target); 

      if (stepper.currentPosition() != target){
          stepper.runSpeedToPosition();   //doesnt implement acceleration    
      }    
      else {  
          flag = 1;
          Serial.println("Finished moving");
      }
  }           
  digitalWrite(enablePinA, LOW);
  digitalWrite(enablePinB, LOW);  
  
  
}
