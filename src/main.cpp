#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float32_multi_array.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

// Hardware Pin Definitions
#define JOINT1_CS_PIN 33
#define JOINT1_CLK_PIN 34 
#define JOINT1_DO_PIN 35

#define JOINT2_CS_PIN 36
#define JOINT2_CLK_PIN 37
#define JOINT2_DO_PIN 38

#define JOINT3_CS_PIN 39
#define JOINT3_CLK_PIN 40
#define JOINT3_DO_PIN 41

#define POT_1_PIN 16  // right_gimbal_1
#define POT_2_PIN 15  // right_gimbal_2
#define POT_3_PIN 19  // right_gimbal_3
#define HALL_PIN 18   // right_gimbal_0

// Analog voltage supply pins
#define SUPPLY_PIN_1 22 // 3.3V voltage supply
#define SUPPLY_PIN_2 23 // 3.3V voltage supply

// ROS 2 Components
rcl_publisher_t joint_state_publisher;
rcl_publisher_t raw_values_publisher;
sensor_msgs__msg__JointState joint_state_msg;
std_msgs__msg__Float32MultiArray raw_values_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// =====================================================================
// JOINT CALIBRATION DATA
// =====================================================================
struct JointCalibration {
  float min_raw;    // Minimum raw sensor reading
  float max_raw;    // Maximum raw sensor reading
  float min_angle;  // Minimum angle in radians
  float max_angle;  // Maximum angle in radians
  float offset_deg; // Joint zero position offset in degrees
  float exp_a, exp_b, exp_c, exp_d; // For hall effect
};

JointCalibration calib[7] = {
  // Encoder Joints (12-bit absolute encoders)
  {0, 4095, 0, 2*M_PI, 148.5},  // right_joint_1
  {0, 4095, 0, 2*M_PI, 305.0},   // right_joint_2
  {0, 4095, 0, 2*M_PI, 26.5},    // right_joint_3
  
  // Potentiometer Joints
  {94, 860, -125.0*M_PI/180.0, 125.0*M_PI/180.0, 0},  // right_gimbal_3
  {72, 930, -40.0*M_PI/180.0, 40.0*M_PI/180.0, 0},    // right_gimbal_2
  {20, 910, -40.0*M_PI/180.0, 40.0*M_PI/180.0, 0},    // right_gimbal_1
  
  // Hall Effect (right_gimbal_0) - now 0-5°
  {555, 973, 0, 5.0*M_PI/180.0, 0, 31.46, 0.000225, -4.767e5, -0.0189}
};
// =====================================================================

// Error handling macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while(1) { delay(100); }
}

void readEncoder(unsigned int *OutData, unsigned int DO, int CSn, unsigned int CLK) {
  *OutData = 0;
  digitalWrite(CSn, LOW);
  delayMicroseconds(1);
  for(int x=0; x<12; x++) {
    digitalWrite(CLK, LOW);
    delayMicroseconds(1);
    digitalWrite(CLK, HIGH);
    delayMicroseconds(1);
    *OutData = (*OutData << 1) | digitalRead(DO);
  }
  digitalWrite(CSn, HIGH);
}

float map_exponential(float x, const JointCalibration &c) {
  return c.exp_a*exp(c.exp_b*x) + c.exp_c*exp(c.exp_d*x);
}

void setup() {
  Serial.begin(921600);
  set_microros_serial_transports(Serial);
  delay(2000);

  // Configure pins
  pinMode(JOINT1_CS_PIN, OUTPUT);
  pinMode(JOINT1_CLK_PIN, OUTPUT);
  pinMode(JOINT1_DO_PIN, INPUT);
  pinMode(JOINT2_CS_PIN, OUTPUT);
  pinMode(JOINT2_CLK_PIN, OUTPUT);
  pinMode(JOINT2_DO_PIN, INPUT);
  pinMode(JOINT3_CS_PIN, OUTPUT);
  pinMode(JOINT3_CLK_PIN, OUTPUT);
  pinMode(JOINT3_DO_PIN, INPUT);
  pinMode(SUPPLY_PIN_1, OUTPUT);
  pinMode(SUPPLY_PIN_2, OUTPUT);
  digitalWrite(JOINT1_CS_PIN, HIGH);
  digitalWrite(JOINT2_CS_PIN, HIGH);
  digitalWrite(JOINT3_CS_PIN, HIGH);
  digitalWrite(SUPPLY_PIN_1, HIGH);
  digitalWrite(SUPPLY_PIN_2, HIGH);

  // Initialize ROS 2
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "mtm_arm_node", "", &support));

  // Initialize publishers
  RCCHECK(rclc_publisher_init_default(
    &joint_state_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "mtm_joint_states"));

  RCCHECK(rclc_publisher_init_default(
    &raw_values_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "mtm_raw_values"));

  // Setup joint state message
  joint_state_msg.name.size = 7;
  joint_state_msg.name.data = (rosidl_runtime_c__String*)malloc(7 * sizeof(rosidl_runtime_c__String));
  
  const char* joint_names[] = {
    "right_joint_1", "right_joint_2", "right_joint_3",
    "right_gimbal_3", "right_gimbal_2", "right_gimbal_1", 
    "right_gimbal_0"
  };
  
  for(int i=0; i<7; i++) {
    joint_state_msg.name.data[i].data = (char*)malloc(20);
    strcpy(joint_state_msg.name.data[i].data, joint_names[i]);
    joint_state_msg.name.data[i].size = strlen(joint_names[i]);
    joint_state_msg.name.data[i].capacity = 20;
  }
  
  joint_state_msg.position.size = 7;
  joint_state_msg.position.data = (double*)malloc(7 * sizeof(double));

  // Setup raw values message
  raw_values_msg.data.size = 7;
  raw_values_msg.data.data = (float*)malloc(7 * sizeof(float));
  raw_values_msg.layout.dim.size = 1;
  raw_values_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*)malloc(sizeof(std_msgs__msg__MultiArrayDimension));
  raw_values_msg.layout.dim.data[0].label.data = (char*)"raw_values";
  raw_values_msg.layout.dim.data[0].label.size = strlen("raw_values");
  raw_values_msg.layout.dim.data[0].size = 7;
  raw_values_msg.layout.dim.data[0].stride = 7;

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
}

void loop() {
  // Read all sensor values
  unsigned int encoderValues[3];
  readEncoder(&encoderValues[0], JOINT1_DO_PIN, JOINT1_CS_PIN, JOINT1_CLK_PIN);
  readEncoder(&encoderValues[1], JOINT2_DO_PIN, JOINT2_CS_PIN, JOINT2_CLK_PIN);
  readEncoder(&encoderValues[2], JOINT3_DO_PIN, JOINT3_CS_PIN, JOINT3_CLK_PIN);
  
  int potValues[3] = {
    analogRead(POT_1_PIN),  // right_gimbal_1
    analogRead(POT_2_PIN),  // right_gimbal_2
    analogRead(POT_3_PIN)   // right_gimbal_3
  };
  int hallValue = analogRead(HALL_PIN);  // right_gimbal_0

  // Publish raw values
  raw_values_msg.data.data[0] = encoderValues[0];
  raw_values_msg.data.data[1] = encoderValues[1];
  raw_values_msg.data.data[2] = encoderValues[2];
  raw_values_msg.data.data[3] = potValues[2];  // gimbal_3
  raw_values_msg.data.data[4] = potValues[1];  // gimbal_2
  raw_values_msg.data.data[5] = potValues[0];  // gimbal_1
  raw_values_msg.data.data[6] = hallValue;     // gimbal_0
  RCSOFTCHECK(rcl_publish(&raw_values_publisher, &raw_values_msg, NULL));

  // Convert to angles (radians first)
  float positions_rad[7];
  
  // Encoder joints with offsets
  for(int i=0; i<3; i++) {
    float normalized = (encoderValues[i] - calib[i].min_raw) / 
                     (calib[i].max_raw - calib[i].min_raw);
    // Apply offset
    float offset_normalized = normalized - (calib[i].offset_deg / 360.0f);
    // Wrap around if needed
    offset_normalized = offset_normalized - floor(offset_normalized);

     // Special case: Invert J1 (right_joint_1)
     if (i == 0) {  // J1 is index 0
      offset_normalized = 1.0f - offset_normalized;  // Invert the direction
    }
    
    positions_rad[i] = offset_normalized * 
                      (calib[i].max_angle - calib[i].min_angle) + 
                      calib[i].min_angle;
  }
  
  // Potentiometer joints
  positions_rad[3] = ((potValues[2] - calib[3].min_raw) / 
                    (calib[3].max_raw - calib[3].min_raw)) *
                    (calib[3].max_angle - calib[3].min_angle) + 
                    calib[3].min_angle;

  positions_rad[4] = ((potValues[1] - calib[4].min_raw) / 
                    (calib[4].max_raw - calib[4].min_raw)) *
                    (calib[4].max_angle - calib[4].min_angle) + 
                    calib[4].min_angle;

  positions_rad[5] = ((potValues[0] - calib[5].min_raw) / 
                    (calib[5].max_raw - calib[5].min_raw)) *
                    (calib[5].max_angle - calib[5].min_angle) + 
                    calib[5].min_angle;
  
  // Hall effect (now 0-5° range)
  positions_rad[6] = map_exponential(hallValue, calib[6]) * 
                    (calib[6].max_angle - calib[6].min_angle) + 
                    calib[6].min_angle;
  // Add 180° flip (π radians) and wrap around if needed
  positions_rad[6] += (M_PI/.75);
  if (positions_rad[6] > 2*M_PI) {
      positions_rad[6] -= 2*M_PI;
  }

  // Convert to degrees for publishing
  for(int i=0; i<7; i++) {
    joint_state_msg.position.data[i] = positions_rad[i] * 180.0 / M_PI;
  }

  // Timestamp with microsecond resolution
  unsigned long now = micros();
  joint_state_msg.header.stamp.sec = now / 1000000;
  joint_state_msg.header.stamp.nanosec = (now % 1000000) * 1000;

  RCSOFTCHECK(rcl_publish(&raw_values_publisher, &raw_values_msg, NULL));
  RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_state_msg, NULL));

  // Remove unnecessary wait
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0)));
}