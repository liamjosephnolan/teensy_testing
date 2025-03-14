#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

// Pin definitions for the right shoulder pitch encoder
#define ENCODER_CS_PIN 9  // Chip Select
#define ENCODER_CLK_PIN 7 // Serial Clock
#define ENCODER_DO_PIN 5  // Serial Data

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while (1) {
    delay(100); // Keep this delay for error handling (non-critical path)
  }
}

// Your encoder reading function
void readEncoder(unsigned int *OutData, unsigned int DO, int CSn, unsigned int CLK, int i) {
  *OutData = 0;  // Reset Output Array.
  digitalWrite(CSn, LOW);
  delayMicroseconds(1);  // Waiting for Tclkfe=500ns Or 16 times NOP
  // Passing 12 times, from 0 to 11
  for (int x = 0; x < 12; x++) {
    digitalWrite(CLK, LOW);
    delayMicroseconds(1);  // Tclk/2_min = 500ns = 8 * 62.5ns
    digitalWrite(CLK, HIGH);
    delayMicroseconds(1);  // Tdo valid, like Tclk/2
    *OutData = (*OutData << 1) | digitalRead(DO);
  }
  digitalWrite(CSn, HIGH);  // Deselects the encoder from reading
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000); // Initial delay for micro-ROS setup (only once)

  // Configure encoder pins
  pinMode(ENCODER_CS_PIN, OUTPUT);
  pinMode(ENCODER_CLK_PIN, OUTPUT);
  pinMode(ENCODER_DO_PIN, INPUT);

  digitalWrite(ENCODER_CS_PIN, HIGH); // Disable the encoder initially

  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // Create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "encoder_angle"));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
}

void loop() {
  // Read the encoder value
  unsigned int encoderValue;
  readEncoder(&encoderValue, ENCODER_DO_PIN, ENCODER_CS_PIN, ENCODER_CLK_PIN, 0);

  // Convert the encoder value to an angle (assuming 12-bit encoder)
  // Adjust the conversion formula based on your encoder's resolution and mechanical setup
  int angle = (encoderValue * 360) / 4096; // Example conversion

  // Set the message data to the angle
  msg.data = angle;

  // Publish the message
  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

  // Spin the executor to process callbacks
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0))); // Spin with no delay
}