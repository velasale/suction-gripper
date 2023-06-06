/*!
 * @file mprls_simpletest.ino
 *
 * A basic test of the sensor with default settings
 * 
 * Designed specifically to work with the MPRLS sensor from Adafruit
 * ----> https://www.adafruit.com/products/3965
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries.  
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#define Serial SerialUSB    //This trick is meant for the Aduino Zero board


#include <Wire.h>
#include "Adafruit_MPRLS.h"

//You don't *need* a reset and EOC pin for most usees, so we set to -1 and don't connect
#define RESET_PIN -1
#define EOC_PIN   -1
#define VALVE 13
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);


/*************************** Base ROS Setup ******************************/
// General ROS packages/nodes
#include <ros.h>
ros::NodeHandle nh;

#include <std_msgs/Float32.h>



/*************************** ROS Services Setup **************************/
#include <std_srvs/Trigger.h>
void closeValveService(const std_srvs::Trigger::Request &req,
                       std_srvs::TriggerResponse &res){
                       res.success = closeValve();
                       }
ros::ServiceServer<std_srvs::Trigger::Request,
    std_srvs::Trigger::Response>
    service_close("closeValve", &closeValveService);


void openValveService(const std_srvs::Trigger::Request &req,
                      std_srvs::TriggerResponse &res){
                      res.success = openValve();
                      }
ros::ServiceServer<std_srvs::Trigger::Request,
    std_srvs::Trigger::Response>
    service_open("openValve", &openValveService);
                      



/*************************** ROS Publishers Setup **************************/
std_msgs::Float32 press_msg;
ros::Publisher publisher_pressure("/gripper/pressure", &press_msg);



// Constants
const int sensorAddress = 0x18;
const int VALVE_DELAY = 10;

// Variables
long publisher_timer;


/**************************** Setup ***************************************/
void setup() {
  // initialize serial:
  Serial.begin(57600);
  
  // Initialize VALVE pin as output
  pinMode(VALVE, OUTPUT);

  // Initialize pressure sensor
  //Wire.begin();
  mpr.begin();

  // ROS stuff
  nh.initNode();
  nh.advertise(publisher_pressure);

  // ROS services
  nh.advertiseService(service_open);
  nh.advertiseService(service_close);

  digitalWrite(VALVE, LOW); 
  
}


/***************************** Loop ****************************************/
void loop() {

  if (millis() > publisher_timer){
    float pressure_hPa = mpr.readPressure();
    delay(10);
    press_msg.data = pressure_hPa;
    publisher_pressure.publish(&press_msg);      
   
    publisher_timer = millis() + 10;    

    Serial.println(pressure_hPa);
  }
  
  nh.spinOnce();  
}

/****************************** Control Functions ***************************/
bool closeValve(){
  bool success = true;
  delay(VALVE_DELAY);
  digitalWrite(VALVE, LOW);
  delay(VALVE_DELAY); 
  return success;  
}


bool openValve(){
  bool success = true;
  delay(VALVE_DELAY);
  digitalWrite(VALVE, HIGH);
  delay(VALVE_DELAY); 
  return success;  
}
