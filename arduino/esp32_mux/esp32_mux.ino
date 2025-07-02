#include <Wire.h>
#include "Adafruit_MPRLS.h"
#include "Adafruit_VL53L0X.h"
#include "Stepper.h"

#define MUX_ADDR 0x70
#define RESET_PIN -1
#define EOC_PIN   -1

#define HALL_IN_PIN 4
#define HALL_OUT_PIN 5


Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

/********   Stepper Motor Variables ********/
/* Pinouts */
const byte enablePinA = 32;
const byte enablePinB = 14;

const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
const int steps = 1500;               // 1475 (70-80mm)    1475(80-90mm)  //For the Mark-10 I tested for every 25 steps
const int stepSpeed = 5;             // 20

/* Speeds */
/* If L298N driver is used, speed should be within the range:
 *    AccelStepper.h library: 450 < x < 1100    units: steps/sec
 *    Stepper.h library:      140 < x < 340     units: rpm      */ 
const int closing_speed = 240;         //240;        // rpm
const int closing_speed_fast = 340;    //340;   // rpm
const int opening_speed = 330;         //330;        // rpm

/* Distances */
const int distance = 58 * (200/8);    // 58mm * (200 steps / 1rev) * (1rev / 8mm)
const int clamp_distance = 15 * (200/8);
const int initial_distance = distance - clamp_distance;
int target;

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 33, 25, 26, 27);
/*********/


void setup() {
  Serial.begin(115200);
  Wire.begin();  // SDA = 21, SCL = 22 on ESP32 by default

  pinMode(HALL_IN_PIN, INPUT);
  pinMode(HALL_OUT_PIN, INPUT);
  
  delay(1000);
  Serial.println("Starting continuous I2C sensor readout...");
}

void loop() {
  // --- Read Hall Effect Sensors ---
  int hall1_state = digitalRead(HALL_IN_PIN);
  int hall2_state = digitalRead(HALL_OUT_PIN);

  Serial.println("Hall Sensor 1: " + String(hall1_state == LOW ? "MAGNET" : "NO MAGNET"));
  Serial.println("Hall Sensor 2: " + String(hall2_state == LOW ? "MAGNET" : "NO MAGNET"));


  for (uint8_t ch = 0; ch < 4; ch++) {
    Serial.println("Selecting channel: " + String(ch));
    pcaselect(ch);
    delay(10);  // Let I2C settle
    yield();    // Avoid watchdog reset

    if (ch < 3) {
      if (!mpr.begin()) {
        Serial.println("❌ MPRLS not found on channel " + String(ch));
      } else {
        float pressure = mpr.readPressure();
        Serial.println("✅ MPRLS pressure on channel " + String(ch) + ": " + String(pressure) + " hPa");
      }
    } else {
      if (!lox.begin()) {
        Serial.println("❌ VL53L0X not found on channel 3");
      } else {
        VL53L0X_RangingMeasurementData_t measure;
        lox.rangingTest(&measure, false);
        if (measure.RangeStatus != 4) {
          Serial.println("✅ VL53L0X distance: " + String(measure.RangeMilliMeter) + " mm");
        } else {
          Serial.println("⚠️ VL53L0X out of range or error");
        }
      }
    }

//    delay(50);  // Optional: slow down readings
  }

    
  delay(100);
  openFingers();
  delay(1000);  
  closeFingers();  
  delay(1000);

  Serial.println("---- End of cycle ----\n");

  
//  delay(1000);  // Pause between full sweeps
}

void pcaselect(uint8_t channel) {
  if (channel > 3) return;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}


void closeFingers() {
  motorSteps(closing_speed_fast, initial_distance);
  delay(100);  
   motorSteps(closing_speed, clamp_distance);  
  delay(100);  
}


void openFingers() {
  motorSteps(opening_speed,-distance);     
  delay(100);
}


void motorSteps(int stp_speed, int stp_distance){    
  digitalWrite(enablePinA, HIGH);
  digitalWrite(enablePinB, HIGH);     
   
  myStepper.setSpeed(stp_speed);
  myStepper.step(stp_distance);
  
  digitalWrite(enablePinA, LOW);
  digitalWrite(enablePinB, LOW);      
  delay(100);            
  
}
