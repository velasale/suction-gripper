#include <AccelStepper.h>
#include <elapsedMillis.h>
#include <Keyboard.h>

#define Serial SerialUSB    //This trick is meant for the Aduino Zero board
#define MotorInterfaceType 4

// The motor resolution is 200steps per revolution. 
// By double checking with Accelstepper, 800steps lead to 16mmm which is 2rev --> 400steps/rev

AccelStepper stepper = AccelStepper(MotorInterfaceType, 9, 10, 11, 12);

const byte enablePinA = 8;
const byte enablePinB = 13;

const int steps = 30 * 50;

const int pos1 = 50 * 100;
const int pos2 = 60 * 100;



elapsedMillis printTime;


void setup() {

  Serial.begin(9600);
  Keyboard.begin();
  
  // put your setup code here, to run once:

  stepper.setMaxSpeed(800);       // steps/second
  stepper.setAcceleration(600);
//  stepper.run();
  
}



void loop() {

  int flag = 0;

  if (Serial.read() == 'u'){
    digitalWrite(enablePinA, HIGH);
    digitalWrite(enablePinB, HIGH);  

    Serial.println("Moving up");

    while (flag == 0){    

//      motorSteps2(800,2000,-pos1);  
      motorSteps2(800,2000,-pos2);    
    
      if (stepper.currentPosition() == -pos2){
        flag = 1;
        Serial.println("Finished moving up");
        }
      }  

       
    digitalWrite(enablePinA, LOW);
    digitalWrite(enablePinB, LOW);  

  }


  if (Serial.read() == 'd'){
    digitalWrite(enablePinA, HIGH);
    digitalWrite(enablePinB, HIGH);  

      Serial.println("Moving down");
  
      while (flag == 0){    
        
        motorSteps2(800,2000,0);
        
      
        if (stepper.currentPosition() == 0){
          flag = 1;
          Serial.println("Finished moving down");
          }
        }   
  
    
    digitalWrite(enablePinA, LOW);
    digitalWrite(enablePinB, LOW);  
  
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


void motorSteps2(int g_speed, int g_accel, int g_pos){
  // Works
   

  stepper.setSpeed(g_speed);
  stepper.moveTo(g_pos);
  
 
  if (stepper.currentPosition() != g_pos){
    stepper.runSpeedToPosition();
    
  }  
  
  
}




void approach_3(int g_speed, int g_accel, int g_pos){
  // This one works
  // Same speed
  
  digitalWrite(enablePinA, HIGH);
  digitalWrite(enablePinB, HIGH);  
  
  delay(10);  
  
  stepper.runToNewPosition(g_pos);
  
    Serial.print(" ");
    Serial.println(stepper.currentPosition());
  
  digitalWrite(enablePinA, LOW);
  digitalWrite(enablePinB, LOW);  
  delay(1000);
  
}



// Run endleslly
//void loop()
//{
//  if (printTime >=1000){
//    printTime = 0;
//    float mSpeed = stepper.speed();
//    Serial.print(mSpeed);
//    Serial.print(" ");
//    Serial.println(stepper.currentPosition());
//  }
//   stepper.runSpeed();
//}
