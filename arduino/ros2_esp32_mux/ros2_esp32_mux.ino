#include <Wire.h>
#include "Adafruit_MPRLS.h"
#include "Adafruit_VL53L0X.h"
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>

#define MUX_ADDR 0x70
#define RESET_PIN -1
#define EOC_PIN   -1

// MPRLS sensors on mux channels 0,1,2
Adafruit_MPRLS mprs[3] = {
  Adafruit_MPRLS(RESET_PIN, EOC_PIN),
  Adafruit_MPRLS(RESET_PIN, EOC_PIN),
  Adafruit_MPRLS(RESET_PIN, EOC_PIN)
};

bool mprls_found[3] = {false, false, false};
bool lox_found = false;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Micro-ROS variables
rcl_publisher_t publisher;
std_msgs__msg__Float32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rcl_timer_t sensor_timer;




// Helper: select mux channel
void pcaselect(uint8_t channel) {
  if (channel > 3) return;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// Helper: round float to 2 decimals
float round2(float val) {
  return roundf(val * 100.0f) / 100.0f;
}

// Timer callback to read sensors and publish
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  
  (void) last_call_time;
  if (timer == NULL) return;

  float data[4] = {-1.0f, -1.0f, -1.0f, -1.0f};

  // Read pressure sensors
  for (uint8_t ch = 0; ch < 3; ch++) {
    if (mprls_found[ch]) {
      pcaselect(ch);
      delay(1);
      data[ch] = mprs[ch].readPressure();
//      data[ch] = round2(data[ch]);
    }
  }

  // Read TOF sensor
  if (lox_found) {
    pcaselect(3);
    delay(1);
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) {
//      data[3] = round2((float)measure.RangeMilliMeter);
      data[3] = (float)measure.RangeMilliMeter;
    }
  }

  // Fill message and publish
  for (size_t i = 0; i < 4; i++) {
    msg.data.data[i] = data[i];
  }
  msg.data.size = 4;

  rcl_publish(&publisher, &msg, NULL);
}

void setup() {
  Serial.begin(2000000);
  
  delay(2000);

  Wire.begin(21, 22);  // Default ESP32 I2C pins: SDA=21, SCL=22
  Wire.setClock(400000);  // Fast I2C (400 kHz)



  // Init MPRLS sensors on mux channels 0-2
  for (uint8_t ch = 0; ch < 3; ch++) {
    pcaselect(ch);
    delay(10);  // allow mux switch settling
    mprls_found[ch] = mprs[ch].begin();
    Serial.printf("MPRLS channel %d init: %s\n", ch, mprls_found[ch] ? "OK" : "FAIL");
  }

  // Init VL53L0X on mux channel 3
  pcaselect(3);
  delay(10);
  lox_found = lox.begin();
  Serial.printf("VL53L0X init: %s\n", lox_found ? "OK" : "FAIL");

  // Micro-ROS setup
  set_microros_transports();

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_node", "", &support);
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "sensor_data"
  );

  // Allocate message data array
  msg.data.data = (float*) malloc(4 * sizeof(float));
  msg.data.capacity = 4;
  msg.data.size = 4;

  // Timer: 50 ms = 20 Hz update
  rclc_timer_init_default(&timer, &support, 50000000, timer_callback);

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);

  Serial.println("Micro-ROS publisher ready.");
}

void loop() {
  rclc_executor_spin(&executor);
}
