/*!
   @file mprls_simpletest.ino

   A basic test of the sensor with default settings

   Designed specifically to work with the MPRLS sensor from Adafruit
   ----> https://www.adafruit.com/products/3965

   These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
   to interface with the breakout.

   Adafruit invests time and resources providing this open source code,
   please support Adafruit and open-source hardware by purchasing
   products from Adafruit!

   Written by Limor Fried/Ladyada for Adafruit Industries.

   MIT license, all text here must be included in any redistribution.

*/

#define Serial SerialUSB    //This trick is meant for the Aduino Zero board

#include <Stepper.h>


/********   Stepper Motor Variables ********/
const byte enablePinA = 8;
const byte enablePinB = 13;

const int stepsPerRevolution = 400;  // change this to fit the number of steps per revolution
const int steps = 1300;              // for your motor

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 9, 10, 11, 12);
/*********/

const bool  USE_ROSSERIAL = true;

#include <Wire.h>
#include <time.h>
#include "Adafruit_MPRLS.h"
#include "Adafruit_VL53L0X.h"

//You don't *need* a reset and EOC pin for most usees, so we set to -1 and don't connect
#define RESET_PIN -1
#define EOC_PIN   -1
#define VALVE 7
#define RESET_PRESSURE_SENSORS 6
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();





/*************************** Base ROS Setup ******************************/
// General ROS packages/nodes
#include <ros.h>
ros::NodeHandle nh;





/*************************** ROS Services Setup **************************/
#include <std_srvs/Trigger.h>
void closeValveService(const std_srvs::Trigger::Request &req,
                       std_srvs::TriggerResponse &res) {
  res.success = closeValve();
}
ros::ServiceServer<std_srvs::Trigger::Request,
    std_srvs::Trigger::Response>
    service_close("closeValve", &closeValveService);


void openValveService(const std_srvs::Trigger::Request &req,
                      std_srvs::TriggerResponse &res) {
  res.success = openValve();
}
ros::ServiceServer<std_srvs::Trigger::Request,
    std_srvs::Trigger::Response>
    service_open("openValve", &openValveService);


void closeFingersService(const std_srvs::Trigger::Request &req,
                       std_srvs::TriggerResponse &res) {
  res.success = closeFingers();
}
ros::ServiceServer<std_srvs::Trigger::Request,
    std_srvs::Trigger::Response>
    service_close_fingers("closeFingers", &closeFingersService);


void openFingersService(const std_srvs::Trigger::Request &req,
                       std_srvs::TriggerResponse &res) {
  res.success = openFingers();
}
ros::ServiceServer<std_srvs::Trigger::Request,
    std_srvs::Trigger::Response>
    service_open_fingers("openFingers", &openFingersService);



    


/*************************** ROS Publishers Setup **************************/
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>


std_msgs::UInt16 press_msg[3];
ros::Publisher publisher_pressure[3] {
  ros::Publisher("/gripper/pressure/sc1", &press_msg[0]),
  ros::Publisher("/gripper/pressure/sc2", &press_msg[1]),
  ros::Publisher("/gripper/pressure/sc3", &press_msg[2])
};


std_msgs::UInt16 dist_msg;
ros::Publisher publisher_distance("/gripper/distance", &dist_msg);



// Constants
const int PCAADR = 0x70;
const int VALVE_DELAY = 10;
const int PUBLISH_DELAY = 2;



/**************************** Setup ***************************************/
void setup() {
  // initialize serial:
  if (!USE_ROSSERIAL){ 
    Serial.begin(57600);     // 57600, 76800, 115200    
  }

  // set the speed at 60 rpm
  myStepper.setSpeed(100);
  
  // Initialize VALVE pin as output
  delay(10);
  pinMode(VALVE, OUTPUT);
  delay(10);
  digitalWrite(VALVE, LOW);
  delay(10);

  // Initialize pressure sensor
  mpr.begin();

  // Initialize ToF sensor
  lox.begin();
  lox.startRangeContinuous(10);

  // Initialize ROS stuff
  if (USE_ROSSERIAL) {

    // Initialize ROS node
    //    nh.getHardware()->setBaud(250000);     //Needs to match the one set in the PC's launch file
    nh.initNode();

    // Initialize ROS publishers
    for (uint8_t i = 0 ; i < 3; i++) {
      nh.advertise(publisher_pressure[i]);
    }
    nh.advertise(publisher_distance);

    // ROS services
    nh.advertiseService(service_open);
    nh.advertiseService(service_close);
    nh.advertiseService(service_open_fingers);
    nh.advertiseService(service_close_fingers);
  }

  // Reset MPRLS sensors
  pinMode(RESET_PRESSURE_SENSORS, HIGH);
  delay(10);
  pinMode(RESET_PRESSURE_SENSORS, LOW);
  delay(10);  
  

}


/***************************** Loop ****************************************/
void loop() {
  
  unsigned long currentMillis;
  VL53L0X_RangingMeasurementData_t measure;
  uint16_t pressure_hPa;
  uint8_t distance;

  //  TODO Parallel

  if (USE_ROSSERIAL) {
    nh.spinOnce();
  }
  else{
    Serial.println("\n......I2C Multiplexing.......");
  } 
  
  for (uint8_t i = 0; i < (3 + 1); i++) {
    // NUM_CUPS +1 is because the i2c multiplexer reads the three pressure sensors AND the
    // time of Flight sensor        

    currentMillis = millis();

    pcaselect(i);
    
    // Measure Pressure in the first three (3) channels 
    if(i<3){
      pressure_hPa = mpr.readPressure();
      press_msg[i].data = pressure_hPa;      

      if (USE_ROSSERIAL) {
        publisher_pressure[i].publish(&press_msg[i]);
        delay(PUBLISH_DELAY);
      }
      else {
        //Print the time in ms just to have an idea of long the sensor takes to measure press.
        Serial.println("[Ch" + String(i) +"] " + "Period: " + String(millis() - currentMillis) + " ms, " + "Pressure: " + String(press_msg[i].data) + " hPa");     
      }
      
    }

    // ... and then measure Distance in the fourth channel
    else {
//      dist_msg.data = lox.readRange();
      
      lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

      if (measure.RangeStatus != 4) {  // phase failures have incorrect data
        dist_msg.data = measure.RangeMilliMeter;        
      } else {
        dist_msg.data = 1e4;
      }

      if (USE_ROSSERIAL){
        publisher_distance.publish(&dist_msg);      
        delay(PUBLISH_DELAY);  
      }
      else {
        Serial.println("[Ch" + String(i) +"] " + "Period: " + String(millis() - currentMillis) + " ms, " + "Distance: " + String(dist_msg.data) + " mm");    
      }      
    }    
    

  }

  

}

/****************************** Control Functions ***************************/
bool closeValve() {
  bool success = true;
  delay(VALVE_DELAY);
  digitalWrite(VALVE, LOW);
  delay(VALVE_DELAY);
  return success;
}


bool openValve() {
  bool success = true;
  delay(VALVE_DELAY);
  digitalWrite(VALVE, HIGH);
  delay(VALVE_DELAY);
  return success;
}


bool openFingers() {
  digitalWrite(enablePinA, HIGH);
  digitalWrite(enablePinB, HIGH);  
  myStepper.step(-steps);

  // With a screw-nut driver, there is no need to leave the driver on
  digitalWrite(enablePinA, LOW);
  digitalWrite(enablePinB, LOW);  
  delay(1500);  
}


bool closeFingers() {
  digitalWrite(enablePinA, HIGH);
  digitalWrite(enablePinB, HIGH);  
  myStepper.step(steps);

  // With a screw-nut driver, there is no need to leave the driver on
  digitalWrite(enablePinA, LOW);
  digitalWrite(enablePinB, LOW);  
  delay(1500);
}



/**************************** I2C Multiplexer *********************************/
// Reference: https://learn.adafruit.com/adafruit-pca9546-4-channel-i2c-multiplexer/arduino
void pcaselect(uint8_t i) {
  if (i > 4) return;

  Wire.beginTransmission(PCAADR);
  Wire.write(1 << i);
  Wire.endTransmission();
}
