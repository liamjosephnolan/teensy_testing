#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h> // Use String message type

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

// Pin definitions for the encoders
#define JOINT1_CS_PIN 0  // Joint 1 Chip Select
#define JOINT1_CLK_PIN 1 // Joint 1 Clock
#define JOINT1_DO_PIN 2  // Joint 1 Data

#define JOINT2_CS_PIN 3  // Joint 2 Chip Select
#define JOINT2_CLK_PIN 4 // Joint 2 Clock
#define JOINT2_DO_PIN 5  // Joint 2 Data

#define JOINT3_CS_PIN 7  // Joint 3 Chip Select
#define JOINT3_CLK_PIN 8 // Joint 3 Clock
#define JOINT3_DO_PIN 9  // Joint 3 Data

// Pin definitions for the gimbal
#define POT_1_PIN 15 
#define HALL_PIN 17
#define POT_2_PIN 19
#define POT_3_PIN 21

// Publishers for joint states and analog values
rcl_publisher_t joint_state_publisher;
rcl_publisher_t analog_value_publisher;

// Messages for joint states and analog values
std_msgs__msg__String joint_state_msg;
std_msgs__msg__String analog_value_msg;

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

// Encoder reading function
void readEncoder(unsigned int *OutData, unsigned int DO, int CSn, unsigned int CLK) {
  *OutData = 0;  // Reset Output Array.
  digitalWrite(CSn, LOW);
  delayMicroseconds(1);  // Waiting for Tclkfe=500ns Or 16 times NOP
  // Passing 12 times, from 0 to 11
  for (int x = 0; x < 12; x++) {
    digitalWrite(CLK, LOW);
    delayMicroseconds(1);  // Tclk/2_min = 500ns = 8 * 62.5
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
  pinMode(JOINT1_CS_PIN, OUTPUT);
  pinMode(JOINT1_CLK_PIN, OUTPUT);
  pinMode(JOINT1_DO_PIN, INPUT);

  pinMode(JOINT2_CS_PIN, OUTPUT);
  pinMode(JOINT2_CLK_PIN, OUTPUT);
  pinMode(JOINT2_DO_PIN, INPUT);

  pinMode(JOINT3_CS_PIN, OUTPUT);
  pinMode(JOINT3_CLK_PIN, OUTPUT);
  pinMode(JOINT3_DO_PIN, INPUT);

  digitalWrite(JOINT1_CS_PIN, HIGH); // Disable the encoder initially
  digitalWrite(JOINT2_CS_PIN, HIGH); // Disable the encoder initially
  digitalWrite(JOINT3_CS_PIN, HIGH); // Disable the encoder initially

  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // Create publishers
  RCCHECK(rclc_publisher_init_default(
    &joint_state_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), // Use String message type
    "mtm_joint_states")); // Publish to the mtm_joint_states topic

  RCCHECK(rclc_publisher_init_default(
    &analog_value_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), // Use String message type
    "mtm_analog_values")); // Publish to the mtm_analog_values topic

  // Initialize the messages
  joint_state_msg.data.data = (char*)malloc(100); // Allocate memory for the string message
  joint_state_msg.data.capacity = 100; // Set the maximum capacity of the string

  analog_value_msg.data.data = (char*)malloc(100); // Allocate memory for the string message
  analog_value_msg.data.capacity = 100; // Set the maximum capacity of the string

  // Initialize the executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
}

void loop() {
  // Read the gimbal values
  int Pot1Value, Pot2Value, Pot3Value, HallValue;
  Pot1Value = analogRead(POT_1_PIN); // Read analog value from POT_1_PIN
  Pot2Value = analogRead(POT_2_PIN); // Read analog value from POT_2_PIN
  Pot3Value = analogRead(POT_3_PIN); // Read analog value from POT_3_PIN
  HallValue = analogRead(HALL_PIN);  // Read analog value from HALL_PIN

  // Format the analog values message
  snprintf(analog_value_msg.data.data, analog_value_msg.data.capacity, "Pot1: %d, Pot2: %d, Pot3: %d, Hall: %d", Pot1Value, Pot2Value, Pot3Value, HallValue);
  analog_value_msg.data.size = strlen(analog_value_msg.data.data); // Update the size of the string

  // Publish the analog values message
  RCSOFTCHECK(rcl_publish(&analog_value_publisher, &analog_value_msg, NULL));

  // Read the encoder values
  unsigned int encoderValue1, encoderValue2, encoderValue3;
  readEncoder(&encoderValue1, JOINT1_DO_PIN, JOINT1_CS_PIN, JOINT1_CLK_PIN);
  readEncoder(&encoderValue2, JOINT2_DO_PIN, JOINT2_CS_PIN, JOINT2_CLK_PIN);
  readEncoder(&encoderValue3, JOINT3_DO_PIN, JOINT3_CS_PIN, JOINT3_CLK_PIN);

  // Convert the encoder values to floating-point angles (assuming 12-bit encoder)
  float angle1 = (encoderValue1 * 360.0f) / 4096.0f; // Joint 1 angle
  float angle2 = (encoderValue2 * 360.0f) / 4096.0f; // Joint 2 angle
  float angle3 = (encoderValue3 * 360.0f) / 4096.0f; // Joint 3 angle

  // Format the joint states message
  snprintf(joint_state_msg.data.data, joint_state_msg.data.capacity, "joint1_angle: %.2f, joint2_angle: %.2f, joint3_angle: %.2f", angle1, angle2, angle3);
  joint_state_msg.data.size = strlen(joint_state_msg.data.data); // Update the size of the string

  // Publish the joint states message
  RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_state_msg, NULL));

  // Spin the executor to process callbacks
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0))); // Spin with a small delay
}