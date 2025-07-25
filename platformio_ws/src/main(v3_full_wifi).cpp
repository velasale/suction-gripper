// src/main.cpp
#include <Arduino.h>
#include <stdio.h>
#include <micro_ros_platformio.h>


#include "driver/gpio.h"
#include "driver/i2c.h"
#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_MPRLS.h>
#include <Adafruit_VL53L0X.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_srvs/srv/set_bool.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int16_multi_array.h>
#include "std_msgs/msg/int16.h"
#include "std_msgs/msg/float32.h"

#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include "BluetoothSerial.h"
BluetoothSerial SerialBT;
#include <WiFi.h>


// === I2C + MUX Configuration ===
#define MUX_ADDR 0x70
// --- MPRLS Pin Definitions (shared per sensor) ---
#define MPRLS_RESET_PIN -1  // Unused or shared, set to -1 if NC
#define MPRLS_EOC_PIN   -1  // Unused or shared, set to -1 if NC

// #define TOF_MUX_CHANNEL 3
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_FREQ 400000

// --- Hardware Pins ---
#define STEP_PIN 26
#define DIR_PIN 25
#define HALL_IN_PIN 4
#define HALL_OUT_PIN 5
#define RELAY_PIN GPIO_NUM_15
#define LED_PIN 2

bool publish_failed = false;
uint32_t last_sensor_read_ms = 0;
const uint32_t SENSOR_PERIOD_MS = 30;
bool ready_to_publish = false;

String COMM_MODE = "wifi_hotspot";  //"wifi_router";  //"wifi_hotspot";  // "serial" or "bluetooth"


// --- Stepper Driver ---
AccelStepper myStepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
// === TOF Sensor ===
Adafruit_VL53L0X tof;
Adafruit_MPRLS mprls_sensors = Adafruit_MPRLS(MPRLS_RESET_PIN, MPRLS_EOC_PIN);
    

// === micro-ROS Entities ===
rcl_node_t node;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
// Publisher and Timer
rcl_publisher_t sensor_pub;
rcl_timer_t sensor_timer;
std_msgs__msg__Int16 tof_msg;
std_msgs__msg__Int16MultiArray sensor_msg;
// std_msgs__msg__Float32 tof_msg;
// std_msgs__msg__Float32MultiArray sensor_msg;

// Services
rcl_service_t stepper_service;
rcl_service_t valve_service;
std_srvs__srv__SetBool_Request stepper_req;
std_srvs__srv__SetBool_Response stepper_res;
std_srvs__srv__SetBool_Request valve_req;
std_srvs__srv__SetBool_Response valve_res;  


// float sensor_data[4]; // [mprls1, mprls2, mprls3, tof]
int16_t sensor_data[4]; // [mprls1, mprls2, mprls3, tof]

// --- Motion Settings ---
const int SPEED_DOWN = 1200;
const int SPEED_UP_BASE = -1200;
const int SPEED_UP_MIN = -600;
const int SLOWDOWN_START = 800;
const int SLOWDOWN_END = 1000;


/* microROS setup */
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)){}}


// microROS system state
enum SystemState {
    AGENT_WAIT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} system_state;


// === Select MUX Channel ===
void select_mux_channel(uint8_t channel) {
  if (channel > 3) return;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delayMicroseconds(250);  // Give time to switch  
}

// --- Function to check for a stable low signal --- 
bool isStableLow(int pin, int samples = 3, int delayMs = 1) {
  for (int i = 0; i < samples; i++) {
    if (digitalRead(pin) == HIGH) {
      return false; // If any sample is HIGH, not stable
    }
    delay(delayMs);
  }
  return true; // All samples were LOW
}

// --- Move Stepper DOWN ---
void move_down() {
  // Serial.println("Moving DOWN...");
  unsigned long start = millis();
  while (!isStableLow(HALL_IN_PIN)) {
    myStepper.setSpeed(SPEED_DOWN);
    myStepper.runSpeed();

    // In case hall sensor fail
    if (millis() - start > 3000) break; // 3s timeout
  }
  myStepper.setCurrentPosition(0);
  // Serial.println("Reached DOWN limit.");
}

// --- Move Stepper UP ---
void move_up() {
  unsigned long start = millis();
  while (!isStableLow(HALL_OUT_PIN)) {
    long pos = abs(myStepper.currentPosition());
    float speed;

    // In case hall sensor fail
    if (millis() - start > 3000) break; // 3s timeout


    if (pos < SLOWDOWN_START) speed = SPEED_UP_BASE;
    else if (pos >= SLOWDOWN_END) speed = SPEED_UP_MIN;
    else {
      float t = float(pos - SLOWDOWN_START) / (SLOWDOWN_END - SLOWDOWN_START);
      speed = SPEED_UP_BASE * (1 - t) + SPEED_UP_MIN * t;
    }
    myStepper.setSpeed(speed);
    myStepper.runSpeed();
  }
  myStepper.setCurrentPosition(0);
}



float read_mprls_sensor(uint8_t mux_channel) {
  select_mux_channel(mux_channel);  
  
  float pressure = mprls_sensors.readPressure();

  if (isnan(pressure)) {
    Serial.printf("Sensor %d read failed\n", mux_channel);
    return -1.0;
  }

  return pressure;
}



// float read_tof_sensor(uint8_t mux_channel) {
//   select_mux_channel(mux_channel);    

//   uint32_t start = millis();
//   while (!tof.isRangeComplete()) {
//     if (millis() - start > 100) {  // 100ms timeout      
//       return -1.0;
//     }
//     delayMicroseconds(50);
//   }

//   uint16_t distance = tof.readRange();
//   if (tof.readRangeStatus() == 0) {
//     return distance;
//   } else {
//     Serial.println("TOF invalid");
//     return -1.0;
//   }
// }



float read_tof_sensor(uint8_t mux_channel) {
  select_mux_channel(mux_channel);  
  uint32_t start = millis();
  while (!tof.isRangeComplete()) {
    if (millis() - start > 10) {  // 10ms timeout
      return -1.0;
    }
    delayMicroseconds(50);
  }
  return tof.readRange();
}




// === Timer Callback ===
void sensor_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer == NULL) return; 
  
  sensor_data[0] = (int16_t)(read_mprls_sensor(0));
  sensor_data[1] = (int16_t)(read_mprls_sensor(1));
  sensor_data[2] = (int16_t)(read_mprls_sensor(2));
  sensor_data[3] = (int16_t)(read_tof_sensor(3));
  
  sensor_msg.data.data = sensor_data; // Assign the sensor data to the message

  // RCSOFTCHECK(rcl_publish(&sensor_pub, &sensor_msg, NULL));  
  rcl_ret_t pub_ret = rcl_publish(&sensor_pub, &sensor_msg, NULL);
  if (pub_ret != RCL_RET_OK) {
      // Serial.printf("Publish failed: %d\n", pub_ret);
      system_state = AGENT_DISCONNECTED;
      publish_failed = true;
  }
}


// Service callback for stepper control
void service_stepper_cb(const void *req, void *res) {
  const std_srvs__srv__SetBool_Request *request = (const std_srvs__srv__SetBool_Request *)req;
  std_srvs__srv__SetBool_Response *response = (std_srvs__srv__SetBool_Response *)res;
  if (request->data) move_up();
  else move_down();
  response->success = true;
  response->message.data = (char *)(request->data ? "Moved up" : "Moved down");
  response->message.size = strlen(response->message.data);
  response->message.capacity = strlen(response->message.data) + 1;
}

// Service callback for valve control
void service_valve_cb(const void *req, void *res) {
  const std_srvs__srv__SetBool_Request *request = (const std_srvs__srv__SetBool_Request *)req;
  std_srvs__srv__SetBool_Response *response = (std_srvs__srv__SetBool_Response *)res;
  gpio_set_level(RELAY_PIN, request->data);
  response->success = true;
  response->message.data = (char *)(request->data ? "Valve ON" : "Valve OFF");
  response->message.size = strlen(response->message.data);
  response->message.capacity = strlen(response->message.data) + 1;
}



bool create_rcl_entities() {
    /* Create the microROS entities */
    // Init allocator
    allocator = rcl_get_default_allocator();
  
    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    
    // create node
    const char* _namespace = "microROS";
    const char* _node_name = "esp32";
    RCCHECK(rclc_node_init_default(&node, _node_name, _namespace, &support));
    
    // Allocate message data array
    // if (sensor_msg.data.data != NULL) {
    //   free(sensor_msg.data.data);
    // }
    sensor_msg.data.data = sensor_data;
    sensor_msg.data.capacity = 4;
    sensor_msg.data.size = 4;


    // Initialize the publisher
    RCCHECK(rclc_publisher_init_default(
      &sensor_pub,
      &node,
      // ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
      "sensor_data"));

    // create services
    rclc_service_init_default(&stepper_service, 
      &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
      "move_stepper");

    rclc_service_init_default(&valve_service,
      &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
      "toggle_valve");


    // // Initialize timer (every 35ms)
    // const unsigned int timer_period_ms = 40; // 35ms = 28.57Hz
    // RCCHECK(rclc_timer_init_default(
    //   &sensor_timer,
    //   &support,
    //   RCL_MS_TO_NS(timer_period_ms),
    //   sensor_timer_callback));
    
    
    rclc_executor_init(&executor, &support.context, 4, &allocator);
    RCCHECK(rclc_executor_add_service(&executor, &stepper_service, &stepper_req, &stepper_res, service_stepper_cb));
    RCCHECK(rclc_executor_add_service(&executor, &valve_service, &valve_req, &valve_res, service_valve_cb));
    // RCCHECK(rclc_executor_add_timer(&executor, &sensor_timer));
    return true;
}

rcl_ret_t destroy_rcl_entities() {
    rcl_ret_t ret_status = RMW_RET_OK;
    /* Destroy the microROS-related entities */
    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    ret_status = rcl_service_fini(&stepper_service, &node);
    ret_status = rcl_service_fini(&valve_service, &node);  
    ret_status = rcl_publisher_fini(&sensor_pub, &node);  
    ret_status = rcl_timer_fini(&sensor_timer);


    ret_status = rcl_node_fini(&node);
    ret_status = rclc_support_fini(&support);

    // free(sensor_msg.data.data);
    sensor_msg.data.data = NULL;
    
    return RCL_RET_OK;
}


void execute_every_n_ms(int64_t ms, SystemState system_state) {
    /* Method for periodically pinging the micro_ros_agent */
    RCL_UNUSED(system_state);
    do {
        static volatile int64_t init = -1;
        if (init == -1) {
            init = uxr_millis();
        }
        if (uxr_millis() - init > ms) {
            // system_state;
            init = uxr_millis();
        }
    }
    while (0);
}


void init_all_sensors() {
  for (uint8_t ch = 0; ch < 3; ch++) {
    select_mux_channel(ch);
    delay(10);
    if (!mprls_sensors.begin()) {
      Serial.printf("MPRLS on channel %d not found\n", ch);
    } else {
      Serial.printf("MPRLS on channel %d initialized\n", ch);
    }
  }

  select_mux_channel(3);
  delay(10);
  if (!tof.begin()) {
    Serial.println("TOF sensor not found on channel 3!");
  } else {
    Serial.println("TOF sensor initialized on channel 3.");
    tof.startRangeContinuous(10);
    tof.setMeasurementTimingBudgetMicroSeconds(5000);
  }
}


void setup() {

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Serial.begin(115200);
  // Serial.begin(921600);
  Serial.begin(2000000);
  delay(500); // Wait for Serial to stabilize  
    
  // --- Wifi through Router ---
  if (COMM_MODE == "wifi_router"){
    char* ssid = "sas-network";
    char* password = "marinerobotics"; 
    // IPAddress agent_ip(192,168,0,123);  // Laptop-Router through LAN    
    IPAddress agent_ip(192,168,0,101);  // Laptop-Router through wifi

    int agent_port = 8888;
    set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
  } 
   

  // --- Wifi through laptop's hotspot ---
  if (COMM_MODE == "wifi_hotspot") {
    char* ssid = "alejos";
    char* password = "harvesting";
    IPAddress agent_ip(10,42,0,1);  // <-- Replace with your ROS 2 computer IP
    int agent_port = 8888;
    set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
  }

  

  // //--- Transports through Bluetooth ---
  // if (COMM_MODE == "bluetooth") {
  //   set_microros_serial_transports(SerialBT);
  //   SerialBT.begin("ESP32_BT");  // You can name this as you wish
  //   while (!SerialBT.hasClient()) {
  //     delay(100);
  //     Serial.print(".");
  //   }
  //   Serial.println("\nBluetooth connected.");
  // }

  // // --- Transports through Serial Cable ---
  // if (COMM_MODE == "serial"){
  //   set_microros_serial_transports(Serial);
  //   Serial.println("Serial transport initialized.");
  // }


  // GPIO setup  
  gpio_pad_select_gpio(RELAY_PIN);
  gpio_set_direction(RELAY_PIN, GPIO_MODE_OUTPUT); 

  pinMode(HALL_IN_PIN, INPUT_PULLUP);
  pinMode(HALL_OUT_PIN, INPUT_PULLUP);

  // Stepper
  myStepper.setMaxSpeed(2000);     // steps per second
  myStepper.setAcceleration(250);  // steps/sec^2
  myStepper.setCurrentPosition(0);  // Set starting position to zero

  // I2C Init
  Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQ);

  init_all_sensors();         
  
  // Initial state
  system_state = AGENT_WAIT;
}

void update_led() {
    static uint32_t last_toggle_time = 0;
    static bool led_on = false;
    uint32_t now = millis();

    int blink_interval;
    switch (system_state) {
        case AGENT_WAIT:         blink_interval = 100;  break; // Fast blink
        case AGENT_AVAILABLE:    blink_interval = 200;  break; // Medium blink
        case AGENT_CONNECTED:    blink_interval = 1000; break; // Slow blink
        case AGENT_DISCONNECTED: blink_interval = 100;  break; // Very fast blink
        default:                 blink_interval = 100; break;
    }

    if (now - last_toggle_time > blink_interval) {
        led_on = !led_on;
        digitalWrite(LED_PIN, led_on ? HIGH : LOW);
        last_toggle_time = now;
    }
}

bool should_ping(uint32_t interval_ms) {
    static uint32_t last_ping = 0;
    uint32_t now = millis();
    if ((now - last_ping) >= interval_ms) {
        last_ping = now;
        return true;
    }
    return false;
}

void loop() {

    update_led();
    // Check system state for connection status to micro_ros_agent
    switch (system_state) {

        case AGENT_WAIT: {
            if (should_ping(100)) {
                system_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : AGENT_WAIT;
            }
            // execute_every_n_ms(100, system_state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : AGENT_WAIT);
            break;
          }

        case AGENT_AVAILABLE: {
            system_state = (true == create_rcl_entities()) ? AGENT_CONNECTED : AGENT_WAIT;

            if (system_state == AGENT_WAIT) {
                destroy_rcl_entities();
            }            
            break;
          }        

        case AGENT_CONNECTED: {
            static int ping_fail_count = 0;
            static int publish_fail_count = 0;

            if (should_ping(500)) {
                if (RMW_RET_OK == rmw_uros_ping_agent(350, 1)) {
                    ping_fail_count = 0;
                } else {
                    ping_fail_count++;
                    if (ping_fail_count >= 3) {
                        system_state = AGENT_DISCONNECTED;
                        ping_fail_count = 0;
                    }
                }
            }

            uint32_t now = millis();
            if (now - last_sensor_read_ms >= SENSOR_PERIOD_MS) {
              last_sensor_read_ms = now;

              sensor_data[0] = (int16_t)(read_mprls_sensor(0));
              sensor_data[1] = (int16_t)(read_mprls_sensor(1));
              sensor_data[2] = (int16_t)(read_mprls_sensor(2));
              sensor_data[3] = (int16_t)min(read_tof_sensor(3), 300.0f);    // Limit TOF to 300mm for safety
              ready_to_publish = true;
            }

            if (ready_to_publish) {
              sensor_msg.data.data = sensor_data;
              rcl_ret_t pub_ret = rcl_publish(&sensor_pub, &sensor_msg, NULL);
              if (pub_ret != RCL_RET_OK) {
                publish_fail_count++;
                if (publish_fail_count >= 3) {
                  system_state = AGENT_DISCONNECTED;
                  publish_fail_count = 0;
                }
              } else {
                publish_fail_count = 0;  // Reset on success
              }
              ready_to_publish = false;
            }

            // Run executor callbacks (services, etc.)
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));           

            break;
          }

        case AGENT_DISCONNECTED: {
            destroy_rcl_entities();
            system_state = AGENT_WAIT;
            break;
        }
    }   
}
