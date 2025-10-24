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
#include "esp_wifi.h"
#include "esp_heap_caps.h"

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
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_FREQ 400000     // 400 KHz
#define MUX_ADDR 0x70
// MPRLS Pin Definitions (shared per sensor)
#define MPRLS_RESET_PIN -1  // Unused or shared, set to -1 if NC
#define MPRLS_EOC_PIN   -1  // Unused or shared, set to -1 if NC


// === Hardware Pins ===
#define STEP_PIN GPIO_NUM_26
#define DIR_PIN GPIO_NUM_25
#define HALL_IN_PIN GPIO_NUM_4
#define HALL_OUT_PIN GPIO_NUM_5
#define RELAY_PIN GPIO_NUM_15
#define LED_PIN GPIO_NUM_2
#define ENABLE_MOTOR_PIN GPIO_NUM_14
// #define TOF_XSHUT_PIN GPIO_NUM_33   // <--- optional, only if VL53L0X XSHUT connected
// #define TOF_GPIO_PIN GPIO_NUM_32

bool publish_failed = false;
uint32_t last_sensor_read_ms = 0;
const uint32_t SENSOR_PERIOD_MS = 40;
bool ready_to_publish = false;


// COMM_MODEs: "wifi_router", "wifi_hotspot", "serial", "bluetooth"
String COMM_MODE = "wifi_hotspot";

// === Hardware objects ===
AccelStepper myStepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
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
// Services
rcl_service_t stepper_service;
rcl_service_t valve_service;
std_srvs__srv__SetBool_Request stepper_req;
std_srvs__srv__SetBool_Response stepper_res;
std_srvs__srv__SetBool_Request valve_req;
std_srvs__srv__SetBool_Response valve_res;  

int16_t sensor_data[4]; // [mprls1, mprls2, mprls3, tof]

// --- Motion Settings ---
const int STEP_MODE = 4;          // Stepper Driver https://www.pololu.com/product/2133 DRV8825 configure at 1/4 step.
const int SPEED_DOWN = 1200 * STEP_MODE;
const int SPEED_UP_BASE = -1200 * STEP_MODE;
const int SPEED_UP_MIN = -600 * STEP_MODE;
const int SLOWDOWN_START = 800 * STEP_MODE;
const int SLOWDOWN_END = 1000 * STEP_MODE;
const int STEPS_PER_MM = 25;

// --- Services variales --- 
char stepper_msg_buf[3]; // "Up" or "Dn"    , keeps as short as possible
char valve_msg_buf[4];   // "ON" or "OFF"   , keeps as short as possible

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


enum StepperState {
    STEPPER_IDLE,
    STEPPER_MOVING_DOWN,
    STEPPER_MOVING_UP
};
volatile StepperState stepper_state = STEPPER_IDLE;
unsigned long stepper_start_time = 0;


void IRAM_ATTR onStepperTimer() {
    if (stepper_state != STEPPER_IDLE) {
        myStepper.runSpeed();  // ISR-safe: no malloc, no Serial, no millis
    }
}



void print_heap_info(const char* tag) {
    multi_heap_info_t info;
    heap_caps_get_info(&info, MALLOC_CAP_8BIT);
    Serial.printf("[%s] Free: %u, Largest block: %u\n", 
                  tag, info.total_free_bytes, info.largest_free_block);
}


// === Select MUX Channel ===
void select_mux_channel(uint8_t channel) {
  if (channel > 3) return;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delayMicroseconds(250);  // Give time to switch  
}

// === Function to check for a stable low signal from Hall Sensors === 
bool isStableLow(int pin, int samples = 3, int delayMs = 1) {
  for (int i = 0; i < samples; i++) {
    if (digitalRead(pin) == HIGH) {
      return false; // If any sample is HIGH, not stable
    }
    delay(delayMs);
  }
  return true; // All samples were LOW
}


void stepperTask(void *pvParameters) {
    const uint32_t idle_timeout_ms = 100; // disable after 1 sec idle
    uint32_t last_move_time = millis();
    bool motor_enabled = false;

    for (;;) {
        switch (stepper_state) {
            case STEPPER_MOVING_DOWN:{

                long pos = abs(myStepper.currentPosition());
                float full_steps;
                float full_dist;

                if (!motor_enabled){
                    gpio_set_level(ENABLE_MOTOR_PIN,0);
                    motor_enabled = true;
                }
                last_move_time = millis();

                if (!isStableLow(HALL_IN_PIN) && millis() - stepper_start_time < 2200) {
                    myStepper.setSpeed(SPEED_DOWN);
                } else {
                    full_steps = pos / STEP_MODE;
                    full_dist = full_steps / STEPS_PER_MM;
                    // Serial.printf("Distance Moving-Down: %.2f steps, %.2f mm \n", full_steps, full_dist);
                    stepper_state = STEPPER_IDLE;
                    myStepper.setCurrentPosition(0);
                }
                break;
              }

            case STEPPER_MOVING_UP:{

                long pos = abs(myStepper.currentPosition());
                float full_steps;
                float full_dist;

                if (!motor_enabled){
                    gpio_set_level(ENABLE_MOTOR_PIN,0);
                    motor_enabled = true;
                }

                if (!isStableLow(HALL_OUT_PIN) && millis() - stepper_start_time < 2200) {
                    pos = abs(myStepper.currentPosition());
                    float speed;
                    if (pos < SLOWDOWN_START) speed = SPEED_UP_BASE;
                    else if (pos >= SLOWDOWN_END) speed = SPEED_UP_MIN;
                    else {
                        float t = float(pos - SLOWDOWN_START) / (SLOWDOWN_END - SLOWDOWN_START);
                        speed = SPEED_UP_BASE * (1 - t) + SPEED_UP_MIN * t;
                    }
                    myStepper.setSpeed(speed);
                } else {
                    full_steps = pos / STEP_MODE;
                    full_dist = full_steps / STEPS_PER_MM;
                    // Serial.printf("Distance Moving-Up: %.2f steps, %.2f mm \n", full_steps, full_dist);
                    stepper_state = STEPPER_IDLE;                    
                    myStepper.setCurrentPosition(0);
                }
                break;
              }

            case STEPPER_IDLE:
            default:
                // Disable motor if idle for too long
                if (motor_enabled && millis() - last_move_time > idle_timeout_ms) {
                    gpio_set_level(ENABLE_MOTOR_PIN, 1); // disable motor
                    motor_enabled = false;
                }
                break;
        }
        vTaskDelay(1); // Yield to keep Wi-Fi/micro-ROS happy
    }
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
  
  rcl_ret_t pub_ret = rcl_publish(&sensor_pub, &sensor_msg, NULL);
  if (pub_ret != RCL_RET_OK) {
      // Serial.printf("Publish failed: %d\n", pub_ret);
      system_state = AGENT_DISCONNECTED;
      publish_failed = true;
  }
}


// // === Service callback for stepper ===
// void service_stepper_cb(const void *req, void *res) {
//     const std_srvs__srv__SetBool_Request *request = (const std_srvs__srv__SetBool_Request *)req;
//     std_srvs__srv__SetBool_Response *response = (std_srvs__srv__SetBool_Response *)res;

//     if (request->data) {
//         stepper_state = STEPPER_MOVING_UP;
//     } else {
//         stepper_state = STEPPER_MOVING_DOWN;
//     }
//     stepper_start_time = millis();

//     response->success = true;
//     response->message.data = (char *)(request->data ? "Up" : "Dn");
//     response->message.size = strlen(response->message.data);
//     response->message.capacity = response->message.size + 1;
// }


void service_stepper_cb(const void *req, void *res) {
    const auto *request = (const std_srvs__srv__SetBool_Request *)req;
    auto *response = (std_srvs__srv__SetBool_Response *)res;

    stepper_state = request->data ? STEPPER_MOVING_UP : STEPPER_MOVING_DOWN;
    stepper_start_time = millis();

    response->success = true;
    strcpy(stepper_msg_buf, request->data ? "Up" : "Dn");
    response->message.data = stepper_msg_buf;
    response->message.size = strlen(stepper_msg_buf);
    response->message.capacity = sizeof(stepper_msg_buf);
}


// === Service callback for valve control ===
// void service_valve_cb(const void *req, void *res) {
//   const std_srvs__srv__SetBool_Request *request = (const std_srvs__srv__SetBool_Request *)req;
//   std_srvs__srv__SetBool_Response *response = (std_srvs__srv__SetBool_Response *)res;

//   gpio_set_level(RELAY_PIN, request->data);

//   response->success = true;
//   response->message.data = (char *)(request->data ? "Air ON" : "Air OFF");
//   response->message.size = strlen(response->message.data);
//   response->message.capacity = strlen(response->message.data) + 1;
// }


void service_valve_cb(const void *req, void *res) {
    const auto *request = (const std_srvs__srv__SetBool_Request *)req;
    auto *response = (std_srvs__srv__SetBool_Response *)res;
    
    gpio_set_level(RELAY_PIN, request->data);       

    response->success = true;
    strcpy(valve_msg_buf, request->data ? "ON" : "OFF");
    response->message.data = valve_msg_buf;
    response->message.size = strlen(valve_msg_buf);
    response->message.capacity = sizeof(valve_msg_buf);
}



// === Destroy all ROS client entities ===
rcl_ret_t destroy_rcl_entities() {   

    rcl_ret_t final_status = RCL_RET_OK;

    rcl_ret_t rc;

    rc = rcl_service_fini(&stepper_service, &node);
    if(rc != RCL_RET_OK) final_status = rc;

    rc = rcl_service_fini(&valve_service, &node);
    if(rc != RCL_RET_OK) final_status = rc;

    rc = rcl_publisher_fini(&sensor_pub, &node);
    if(rc != RCL_RET_OK) final_status = rc;

    rc = rcl_timer_fini(&sensor_timer);
    if(rc != RCL_RET_OK) final_status = rc;

    rc = rcl_node_fini(&node);
    if(rc != RCL_RET_OK) final_status = rc;

    rc = rclc_support_fini(&support);
    if(rc != RCL_RET_OK) final_status = rc;    
  
    sensor_msg.data.data = NULL;

    print_heap_info("Before destroy");
    
    return final_status;
}



// === Method to initialize all sensors ===
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


void reset_i2c_and_sensors() {
    Serial.println("Resetting I2C bus and sensors...");

    // Stop all current I2C transactions
    Wire.end();
    delay(50);

    // Re-init I2C bus
    Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQ);
    delay(50);

    #ifdef TOF_XSHUT_PIN
    // Optional hardware reset of ToF (if XSHUT pin is used)
    pinMode(TOF_GPIO_PIN, INPUT_PULLUP);
    pinMode(TOF_XSHUT_PIN, OUTPUT);
    digitalWrite(TOF_XSHUT_PIN, LOW);
    delay(10);
    digitalWrite(TOF_XSHUT_PIN, HIGH);
    delay(10);
    #endif


    // Re-initialize all sensors
    init_all_sensors();
}




bool create_rcl_entities() {

    reset_i2c_and_sensors();
    
    /* Create the microROS entities */
    // Init allocator

    rcl_ret_t rc;
    allocator = rcl_get_default_allocator();
  
    // Init support
    rc = rclc_support_init(&support, 0, NULL, &allocator);
    if(rc != RCL_RET_OK){
      Serial.printf("Error rclc_support_init failed (%d)\n", rc);
      return false;
    }
        
    // Create node
    const char* _namespace = "microROS";
    const char* _node_name = "esp32";
    rc = rclc_node_init_default(&node, _node_name, _namespace, &support);
    if (rc != RCL_RET_OK){
      Serial.printf("Error: rclc_node_init_default failed (%d)\n", rc);
      rclc_support_fini(&support);
      return false;
    }
    
    // Prepare message storage
    sensor_msg.data.data = sensor_data;
    sensor_msg.data.capacity = 4;
    sensor_msg.data.size = 4;


    // Initialize the publisher
    rc = rclc_publisher_init_default(
      &sensor_pub,
      &node,
      // ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16MultiArray),
      "sensor_data");
    if (rc != RCL_RET_OK){
      Serial.printf("Error: publibsher init failed (%d)\n", rc);
      (void) rcl_node_fini(&node);
      rclc_support_fini(&support);
      return false;
    }

    // Stepper Service
    rc = rclc_service_init_default(&stepper_service,      
      &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
      "move_stepper");
    if (rc != RCL_RET_OK){
      Serial.printf("Error: stepper service init failed (%d)\n", rc);
      rcl_publisher_fini(&sensor_pub, &node);
      rcl_node_fini(&node);
      rclc_support_fini(&support);
      return false;
    }

    // Valve service
    rc = rclc_service_init_default(&valve_service,
      &node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
      "toggle_valve");
    if (rc != RCL_RET_OK){
      Serial.printf("Error: valve service init failed (%d)\n", rc);
      rcl_service_fini(&stepper_service, &node);
      rcl_publisher_fini(&sensor_pub, &node);
      rcl_node_fini(&node);
      rclc_support_fini(&support);
      return false;
    }


    // // Initialize timer (every 35ms)
    // const unsigned int timer_period_ms = 40; // 35ms = 28.57Hz
    // RCCHECK(rclc_timer_init_default(
    //   &sensor_timer,
    //   &support,
    //   RCL_MS_TO_NS(timer_period_ms),
    //   sensor_timer_callback));
    
    // Executor
    rc = rclc_executor_init(&executor,
      &support.context,
      4,
      &allocator);
    if (rc != RCL_RET_OK){
      Serial.printf("Error: executor init failed (%d)\n", rc);
      rcl_service_fini(&valve_service, &node);
      rcl_service_fini(&stepper_service, &node);
      rcl_publisher_fini(&sensor_pub, &node);
      rcl_node_fini(&node);
      rclc_support_fini(&support);
      return false;
    }

    // Add services
    rc = rclc_executor_add_service(&executor, &stepper_service, &stepper_req, &stepper_res, service_stepper_cb);
    if (rc != RCL_RET_OK){
      Serial.printf("Error: add stepper serviec to executor failed (%d)\n", rc);
      destroy_rcl_entities();
      return false;
    }

    rc = rclc_executor_add_service(&executor, &valve_service, &valve_req, &valve_res, service_valve_cb);
    if (rc != RCL_RET_OK){
      Serial.printf("Error: add valve service toi executor failed (%d)\n", rc);
      destroy_rcl_entities();
      return false;
    }
    
    return true;
}


// === Microros transports ===
void esp32_laptop_comm(){

  // // === Wifi through Router ===
  // if (COMM_MODE == "wifi_router"){
  //   const char* ssid = "sas-network";
  //   const char* password = "marinerobotics"; 
    
  //   // IPAddress agent_ip(192,168,0,123);  // Laptop-Router through LAN    
  //   IPAddress agent_ip(192,168,0,100);  // Laptop-Router through wifi
  //   int agent_port = 8888;
  //   set_microros_wifi_transports((char*)ssid, (char*)password, agent_ip, agent_port);
  //   while (WiFi.status() != WL_CONNECTED) {
  //     delay(50);
  //     Serial.print(".");
  //   }
  //   Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
    
  //   WiFi.setSleep(false);  // Keep Wi-Fi fully powered, reduces latency
  //   int ch = WiFi.channel();
  //   esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
  // }    

  // === Wifi through laptop's hotspot ===
  if (COMM_MODE == "wifi_hotspot") {
    const char* ssid = "alejos";
    const char* password = "harvesting";    
    
    IPAddress agent_ip(10,42,0,1);  // <-- Replace with your ROS 2 computer IP
    int agent_port = 8888;
    set_microros_wifi_transports((char*)ssid, (char*)password, agent_ip, agent_port);
    while (WiFi.status() != WL_CONNECTED) {
      delay(50);
      Serial.print(".");
    }
    Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
    
    WiFi.setSleep(false);  // Keep Wi-Fi fully powered, reduces latency
    int ch = WiFi.channel();
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
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
}


void setup() {

  // Serial.begin(115200);
  Serial.begin(921600);
  // Serial.begin(2000000);
  delay(500); // Wait for Serial to stabilize  
  

  // GPIO setup
  gpio_reset_pin(RELAY_PIN);
  gpio_set_direction(RELAY_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(RELAY_PIN, 0); // optional: start OFF

  gpio_reset_pin(HALL_IN_PIN);
  gpio_set_direction(HALL_IN_PIN, GPIO_MODE_INPUT);
  gpio_pullup_en(HALL_IN_PIN);

  gpio_reset_pin(HALL_OUT_PIN);
  gpio_set_direction(HALL_OUT_PIN, GPIO_MODE_INPUT);
  gpio_pullup_en(HALL_OUT_PIN);

  gpio_reset_pin(LED_PIN);
  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(LED_PIN, 0); // LOW

  gpio_reset_pin(ENABLE_MOTOR_PIN);
  gpio_set_direction(ENABLE_MOTOR_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(ENABLE_MOTOR_PIN,1);    // disable motor at startup
   
  // ROS2 transports setup
  esp32_laptop_comm();     
        

  // Stepper
  myStepper.setMaxSpeed(2000*STEP_MODE);     // steps per second
  myStepper.setAcceleration(250*STEP_MODE);  // steps/sec^2
  myStepper.setCurrentPosition(0);  // Set starting position to zero

  // Start stepper task
  xTaskCreatePinnedToCore(stepperTask, "StepperTask", 4096, NULL, 2, NULL, 1);

  // Timer for precise stepping (0.5 ms = 2 kHz)
  hw_timer_t *stepperTimer = timerBegin(0, 80, true); // 80 prescaler = 1µs ticks
  timerAttachInterrupt(stepperTimer, &onStepperTimer, true);
  timerAlarmWrite(stepperTimer, 250, true); // 500µs
  timerAlarmEnable(stepperTimer);

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
        case AGENT_AVAILABLE:    blink_interval = 100;  break; // Medium blink
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
            break;
          }

        case AGENT_AVAILABLE: {
            system_state = (true == create_rcl_entities()) ? AGENT_CONNECTED : AGENT_WAIT;

            if (system_state == AGENT_CONNECTED) {
              print_heap_info("After create_rcl_entities");
            }

            if (system_state == AGENT_WAIT) {
                destroy_rcl_entities();
            }            
            break;
          }        

        case AGENT_CONNECTED: {
            static int ping_fail_count = 0;
            static int publish_fail_count = 0;
            static uint32_t connected_start = millis();

            if (system_state == AGENT_CONNECTED && should_ping(5000)) {
              print_heap_info("5s check");
            }

            // If connected for more than 30 sec, check connection
            if (millis() - connected_start > 30000) {
                // Serial.println("\nRestarting connection to keep it fresh...");

                // Quick ping to see if agent is alive
                if (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) {
                    // Serial.println("Agent alive - skipping teardown.");
                    connected_start = millis(); // just reset timer
                    break; // skip full reconnect
                }

                // Serial.println("Agent not responding - reconnecting...");
                destroy_rcl_entities();
                
                // Only run Wi-Fi connect if actually dropped
                if (WiFi.status() != WL_CONNECTED) {
                    WiFi.reconnect(); // faster than starting from scratch
                    uint32_t start = millis();
                    while (WiFi.status() != WL_CONNECTED && millis() - start < 1000) {
                        delay(10); // tight polling
                    }
                    esp_wifi_set_channel(WiFi.channel(), WIFI_SECOND_CHAN_NONE);
                }
                
                system_state = AGENT_WAIT;
                connected_start = millis();
                break;
            } 

            // Ping agent regularly
            if (should_ping(500)) {
                if (RMW_RET_OK == rmw_uros_ping_agent(50, 1)) {      // 800ms timeout
                    ping_fail_count = 0;
                } else {
                    ping_fail_count++;
                    if (ping_fail_count >= 3) {       // 3 consecutive fails -> disconnect
                        system_state = AGENT_DISCONNECTED;
                        ping_fail_count = 0;
                    }
                }
            }

            // Read sensors at fixed period
            uint32_t now = millis();
            if (now - last_sensor_read_ms >= SENSOR_PERIOD_MS) {
              last_sensor_read_ms = now;
              sensor_data[0] = (int16_t)(read_mprls_sensor(0));
              sensor_data[1] = (int16_t)(read_mprls_sensor(1));
              sensor_data[2] = (int16_t)(read_mprls_sensor(2));
              sensor_data[3] = (int16_t)min(read_tof_sensor(3), 300.0f);    // Limit TOF to 300mm for safety
              ready_to_publish = true;
            }

            // Publish if ready
            if (ready_to_publish) {
              sensor_msg.data.data = sensor_data;
              // delay(2); // yield to Wi-Fi stack
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
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)); 
            break;
          }

        case AGENT_DISCONNECTED: {
            // Serial.println("Agent disconnected - cleaning up.");
            destroy_rcl_entities();

            reset_i2c_and_sensors();

            // If Wi-Fi is actually down, do minimal reconnect
            if (WiFi.status() != WL_CONNECTED) {
                // Serial.println("Wi-Fi lost - reconnecting...");
                WiFi.reconnect(); // much faster than full esp32_laptop_comm()
                uint32_t start = millis();
                while (WiFi.status() != WL_CONNECTED && millis() - start < 1000) {
                    delay(10); // tight polling
                }
                esp_wifi_set_channel(WiFi.channel(), WIFI_SECOND_CHAN_NONE);
            } else {
                // Serial.println("Wi-Fi still connected - skipping reconnect.");
            }

            // Back to waiting for agent
            system_state = AGENT_WAIT;
            break;
          }

    }   
}
