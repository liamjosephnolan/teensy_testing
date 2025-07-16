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

// =====================================================================
// HARDWARE PIN DEFINITIONS
// =====================================================================
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
#define SUPPLY_PIN_2 23 // 3.4V voltage supply

// =====================================================================
// ROS 2 COMPONENTS
// =====================================================================
rcl_publisher_t joint_state_publisher;
rcl_publisher_t raw_values_publisher;
rcl_publisher_t target_pose_publisher; // NEW: Publisher for target_pose
sensor_msgs__msg__JointState joint_state_msg;
std_msgs__msg__Float32MultiArray raw_values_msg;
sensor_msgs__msg__JointState target_pose_msg; // NEW: Message for target_pose

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// =====================================================================
// JOINT CALIBRATION DATA (for mtm_joint_states)
// =====================================================================
// CORRECTED: Simplified struct
struct JointCalibration {
  float min_raw;    // Minimum raw sensor reading
  float max_raw;    // Maximum raw sensor reading
  float min_angle;  // Minimum angle in radians
  float max_angle;  // Maximum angle in radians
  float offset_deg; // Joint zero position offset in degrees
};

// CORRECTED: Updated calib array
JointCalibration calib[7] = {
  // Encoder Joints (12-bit absolute encoders)
  // New J1 offset_deg calculated: (1.0 - (1680.0/4095.0)) * 360.0 = 212.3078
  {0, 4095, 0, 2*PI, 148.31},  // right_joint_1
  // New J2 offset_deg calculated: (1145.0/4095.0) * 360.0 = 100.6593
  {0, 4095, 0, 2*PI, 100.66},   // right_joint_2
  // New J3 offset_deg calculated: (1920.0/4095.0) * 360.0 = 168.7910
  {0, 4095, 0, 2*PI, 168.79},    // right_joint_3

  // Potentiometer Joints (updated min/max raw values)
  {100, 860, -125.0*PI/180.0, 125.0*PI/180.0, 0},  // right_gimbal_3 (min_raw: 100, max_raw: 860)
  {3, 915, -40.0*PI/180.0, 40.0*PI/180.0, 0},    // right_gimbal_2 (min_raw: 3, max_raw: 915)
  {71, 971, -40.0*PI/180.0, 40.0*PI/180.0, 0},    // right_gimbal_1 (min_raw: 71, max_raw: 971)

  // Hall Effect (right_gimbal_0)
  // Effective range: 580-1000. Below 580 is clamped to 0.
  {560, 1000, -10 * PI / 180.0, 45.0 * PI / 180.0, 0}
};
// =====================================================================
// KINEMATICS DATA & FUNCTIONS (from old code)
// These are used specifically for calculating the /target_pose
// =====================================================================
struct EncoderPinsKinematics {
  int cs;
  int clk;
  int dout;
};

// Define pins for each encoder used in kinematics calculation
EncoderPinsKinematics encoders_kin[3] = {
  {JOINT1_CS_PIN, JOINT1_CLK_PIN, JOINT1_DO_PIN},  // Encoder 1 (right_joint_1)
  {JOINT2_CS_PIN, JOINT2_CLK_PIN, JOINT2_DO_PIN},  // Encoder 2 (right_joint_2)
  {JOINT3_CS_PIN, JOINT3_CLK_PIN, JOINT3_DO_PIN}   // Encoder 3 (right_joint_3)
};

// UPDATED: Encoder offsets in ticks, from the new code snippet
float encoder_offsets_kin[3] = {1712, 1135, 1931};

#define TICKS_PER_REV 4096

// Function to read 12-bit absolute encoder for kinematics
uint16_t readEncoderForKinematics(const EncoderPinsKinematics& ep) {
  uint16_t value = 0;

  digitalWrite(ep.cs, LOW);
  delayMicroseconds(1);

  for (int i = 0; i < 12; i++) {
    digitalWrite(ep.clk, HIGH);
    delayMicroseconds(1);

    value <<= 1;
    if (digitalRead(ep.dout)) {
      value |= 1;
    }

    digitalWrite(ep.clk, LOW);
    delayMicroseconds(1);
  }

  digitalWrite(ep.cs, HIGH);
  return value;
}

// Convert encoder ticks to radians for kinematics
float encoderToRadKin(int16_t ticks) { // Changed name to avoid conflict and parameter to int16_t
  return (2.0f * PI * ticks) / (float)TICKS_PER_REV;
}

// Compute Denavit-Hartenberg (DH) transformation matrix
void computeDHMatrix(float theta, float d, float a, float alpha, float T[4][4]) {
  T[0][0] = cos(theta);             T[0][1] = -sin(theta) * cos(alpha);  T[0][2] = sin(theta) * sin(alpha);   T[0][3] = a * cos(theta);
  T[1][0] = sin(theta);             T[1][1] = cos(theta) * cos(alpha);   T[1][2] = -cos(theta) * sin(alpha);  T[1][3] = a * sin(theta);
  T[2][0] = 0;                      T[2][1] = sin(alpha);                T[2][2] = cos(alpha);                T[2][3] = d;
  T[3][0] = 0;                      T[3][1] = 0;                         T[3][2] = 0;                         T[3][3] = 1;
}

// Multiply two 4x4 matrices
void multiplyMatrix(float A[4][4], float B[4][4], float result[4][4]) {
  float temp[4][4];
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      temp[i][j] = 0;
      for (int k = 0; k < 4; k++) {
        temp[i][j] += A[i][k] * B[k][j];
      }
    }
  }
  // Copy result from temp to result
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      result[i][j] = temp[i][j];
    }
  }
}

// =====================================================================
// ERROR HANDLING
// =====================================================================
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while(1) { delay(100); }
}

// =====================================================================
// ENCODER & SENSOR READING FUNCTIONS (for mtm_joint_states)
// =====================================================================

// Function to read 12-bit absolute encoder for mtm_joint_states and raw_values
// Note: This is separate from readEncoderForKinematics due to different parameter types
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

// CORRECTED: The map_exponential function has been removed.

// =====================================================================
// SETUP FUNCTION
// =====================================================================
void setup() {
  Serial.begin(921600);
  set_microros_serial_transports(Serial);
  delay(2000);

  // Configure general I/O pins for encoders
  pinMode(JOINT1_CS_PIN, OUTPUT);
  pinMode(JOINT1_CLK_PIN, OUTPUT);
  pinMode(JOINT1_DO_PIN, INPUT);
  pinMode(JOINT2_CS_PIN, OUTPUT);
  pinMode(JOINT2_CLK_PIN, OUTPUT);
  pinMode(JOINT2_DO_PIN, INPUT);
  pinMode(JOINT3_CS_PIN, OUTPUT);
  pinMode(JOINT3_CLK_PIN, OUTPUT);
  pinMode(JOINT3_DO_PIN, INPUT);

  // Initialize encoder CS and CLK pins (consistent with old kinematics code)
  digitalWrite(JOINT1_CS_PIN, HIGH);
  digitalWrite(JOINT1_CLK_PIN, LOW); // Set CLK low as per old code's setupEncoderPins
  digitalWrite(JOINT2_CS_PIN, HIGH);
  digitalWrite(JOINT2_CLK_PIN, LOW); // Set CLK low as per old code's setupEncoderPins
  digitalWrite(JOINT3_CS_PIN, HIGH);
  digitalWrite(JOINT3_CLK_PIN, LOW); // Set CLK low as per old code's setupEncoderPins

  // Configure potentiometer and Hall effect pins
  pinMode(POT_1_PIN, INPUT);
  pinMode(POT_2_PIN, INPUT);
  pinMode(POT_3_PIN, INPUT);
  pinMode(HALL_PIN, INPUT);

  // Configure voltage supply pins
  pinMode(SUPPLY_PIN_1, OUTPUT);
  pinMode(SUPPLY_PIN_2, OUTPUT);
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

  // NEW: Initialize target_pose_publisher
  RCCHECK(rclc_publisher_init_default(
    &target_pose_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/target_pose"));

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
  joint_state_msg.velocity.size = 0; // Not used for this message
  joint_state_msg.effort.size = 0;   // Not used for this message


  // Setup raw values message
  raw_values_msg.data.size = 7;
  raw_values_msg.data.data = (float*)malloc(7 * sizeof(float));
  raw_values_msg.layout.dim.size = 1;
  raw_values_msg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*)malloc(sizeof(std_msgs__msg__MultiArrayDimension));
  raw_values_msg.layout.dim.data[0].label.data = (char*)"raw_values";
  raw_values_msg.layout.dim.data[0].label.size = strlen("raw_values");
  raw_values_msg.layout.dim.data[0].size = 7;
  raw_values_msg.layout.dim.data[0].stride = 7;


  // NEW: Setup target_pose_msg
  target_pose_msg.name.size = 3;
  target_pose_msg.name.data = (rosidl_runtime_c__String*)malloc(3 * sizeof(rosidl_runtime_c__String));

  const char* target_pose_joint_names[] = {"x", "y", "z"};
  for(int i=0; i<3; i++) {
      target_pose_msg.name.data[i].data = (char*)malloc(20);
      strcpy(target_pose_msg.name.data[i].data, target_pose_joint_names[i]);
      target_pose_msg.name.data[i].size = strlen(target_pose_joint_names[i]);
      target_pose_msg.name.data[i].capacity = 20;
  }

  target_pose_msg.position.size = 3;
  target_pose_msg.position.data = (double*)malloc(3 * sizeof(double));
  for(int i=0; i<3; i++) {
      target_pose_msg.position.data[i] = 0.0; // Initialize
  }

  // Velocity and Effort are part of JointState, allocate even if not directly used in this publisher
  target_pose_msg.velocity.size = 3;
  target_pose_msg.velocity.data = (double*)malloc(3 * sizeof(double));
  for(int i=0; i<3; i++) {
      target_pose_msg.velocity.data[i] = 0.0; // Initialize
  }

  target_pose_msg.effort.size = 3;
  target_pose_msg.effort.data = (double*)malloc(3 * sizeof(double));
  for(int i=0; i<3; i++) {
      target_pose_msg.effort.data[i] = 0.0; // Initialize
  }

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

}

// =====================================================================
// LOOP FUNCTION
// =====================================================================

// Filtered values for encoders, potentiometers, and hall effect sensor
float filtered_encoder_values[3] = {0, 0, 0};
float filtered_pot_values[3] = {0, 0, 0};
float filtered_hall_value = 0;

// Smoothing factor for the low-pass filter
const float alpha = 0.1; // Adjust this value for desired smoothing

void loop() {
  // Read all sensor values for mtm_joint_states and raw_values
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

  // Publish raw values (unfiltered)
  raw_values_msg.data.data[0] = (float)encoderValues[0];
  raw_values_msg.data.data[1] = (float)encoderValues[1];
  raw_values_msg.data.data[2] = (float)encoderValues[2];
  raw_values_msg.data.data[3] = (float)potValues[2];  // gimbal_3
  raw_values_msg.data.data[4] = (float)potValues[1];  // gimbal_2
  raw_values_msg.data.data[5] = (float)potValues[0];  // gimbal_1
  raw_values_msg.data.data[6] = (float)hallValue;     // gimbal_0
  RCSOFTCHECK(rcl_publish(&raw_values_publisher, &raw_values_msg, NULL));

  // Apply low-pass filter for joint state data
  for (int i = 0; i < 3; i++) {
    filtered_encoder_values[i] = alpha * encoderValues[i] + (1 - alpha) * filtered_encoder_values[i];
  }
  for (int i = 0; i < 3; i++) {
    filtered_pot_values[i] = alpha * potValues[i] + (1 - alpha) * filtered_pot_values[i];
  }
  filtered_hall_value = alpha * hallValue + (1 - alpha) * filtered_hall_value;

  // Convert to angles (radians first) using filtered data for mtm_joint_states
  float positions_rad[7];

  // Encoder joints with offsets
  for (int i = 0; i < 3; i++) {
    float normalized = (filtered_encoder_values[i] - calib[i].min_raw) /
                     (calib[i].max_raw - calib[i].min_raw);
    // Apply offset
    float offset_normalized = normalized - (calib[i].offset_deg / 360.0f);
    // Wrap around if needed
    offset_normalized = offset_normalized - floor(offset_normalized);

    // Special case: Invert J1 (right_joint_1)
    if (i == 0) {  // J1 is index 0
      offset_normalized = 1.0f - offset_normalized;  // Invert the direction
    }

    positions_rad[i] = offset_normalized * (calib[i].max_angle - calib[i].min_angle) +
                      calib[i].min_angle;
  }

  // Potentiometer joints
  positions_rad[3] = ((filtered_pot_values[2] - calib[3].min_raw) /
                    (calib[3].max_raw - calib[3].min_raw)) *
                    (calib[3].max_angle - calib[3].min_angle) +
                    calib[3].min_angle;

  positions_rad[4] = ((filtered_pot_values[1] - calib[4].min_raw) /
                    (calib[4].max_raw - calib[4].min_raw)) *
                    (calib[4].max_angle - calib[4].min_angle) +
                    calib[4].min_angle;

  positions_rad[5] = ((filtered_pot_values[0] - calib[5].min_raw) /
                    (calib[5].max_raw - calib[5].min_raw)) *
                    (calib[5].max_angle - calib[5].min_angle) +
                    calib[5].min_angle;

  // CORRECTED: New linear mapping logic for the Hall effect sensor
  // Hall effect (right_gimbal_0) from 0 to 45 degrees
  // Values under the minimum raw value (580) are set to 0 degrees.
  if (filtered_hall_value < calib[6].min_raw) {
    positions_rad[6] = calib[6].min_angle; // Set to 0 rad
  } else {
    // Linearly map the value from the effective range [580, 1000] to the angle range
    float normalized_value = (filtered_hall_value - calib[6].min_raw) /
                           (calib[6].max_raw - calib[6].min_raw);

    // Clamp the value to 1.0 to prevent exceeding the max angle
    if (normalized_value > 1.0f) {
      normalized_value = 1.0f;
    }

    positions_rad[6] = normalized_value * (calib[6].max_angle - calib[6].min_angle) +
                       calib[6].min_angle;
  }

  // Convert to degrees for publishing mtm_joint_states (as per your current code)
  for (int i = 0; i < 7; i++) {
    joint_state_msg.position.data[i] = positions_rad[i] * 180.0 / PI;
  }

  // Timestamp with microsecond resolution for mtm_joint_states
  unsigned long now = micros();
  joint_state_msg.header.stamp.sec = now / 1000000;
  joint_state_msg.header.stamp.nanosec = (now % 1000000) * 1000;

  RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_state_msg, NULL));

  // --- KINEMATICS CALCULATION FOR /target_pose ---
  // Read encoder ticks for J1, J2, J3 using the kinematics-specific reader
  // Cast to int16_t for use with encoderToRadKin as per the new code's function signature
  int16_t enc1_ticks_kin = (int16_t)readEncoderForKinematics(encoders_kin[0]);
  int16_t enc2_ticks_kin = (int16_t)readEncoderForKinematics(encoders_kin[1]);
  int16_t enc3_ticks_kin = (int16_t)readEncoderForKinematics(encoders_kin[2]);

  // Convert to radians and apply UPDATED offsets from kinematics code
  float q1_kin = -encoderToRadKin(enc1_ticks_kin - encoder_offsets_kin[0]);
  float q2_kin = -encoderToRadKin(enc2_ticks_kin - encoder_offsets_kin[1]);
  float q3_kin = encoderToRadKin(enc3_ticks_kin - encoder_offsets_kin[2]);

  // Apply q3 offset as per kinematics code
  q3_kin -= PI / 2.0f; // Use f suffix for float literal

  // DH parameters - use 'f' suffix for float literals to avoid double promotion
  float d_dh[3]     = {  0.0f,    0.0f,  -40.0f };
  float a_dh[3]     = { 45.0f,  217.5f, 217.5f };
  float alpha_dh[3] = { PI/2.0f,  -PI/2.0f, -PI/2.0f };
  // Note: theta[2] in the old code became q3_kin - PI/2.0f.
  // q3_kin already had PI/2.0f subtracted, so this effectively subtracts PI from initial q3.
  float theta_dh[3] = { q1_kin, q2_kin, q3_kin - PI/2.0f };

  // Compute transformation matrices
  float T01_dh[4][4], T12_dh[4][4], T23_dh[4][4];
  float T02_dh[4][4], T03_dh[4][4];

  computeDHMatrix(theta_dh[0], d_dh[0], a_dh[0], alpha_dh[0], T01_dh);
  computeDHMatrix(theta_dh[1], d_dh[1], a_dh[1], alpha_dh[1], T12_dh);
  computeDHMatrix(theta_dh[2], d_dh[2], a_dh[2], alpha_dh[2], T23_dh);

  multiplyMatrix(T01_dh, T12_dh, T02_dh);
  multiplyMatrix(T02_dh, T23_dh, T03_dh);

  // Extract XYZ and apply global offsets from kinematics code
  // Convert from millimeters (used in DH params) to meters (standard ROS 2 units)
  float x_pos = (T03_dh[0][3] - 262.5f) / 1000.0f;
  float y_pos = (T03_dh[2][3] + 40.0f) / 1000.0f;
  float z_pos = (T03_dh[1][3] + 217.5f) / 1000.0f;

  // Populate target_pose_msg
  target_pose_msg.position.data[0] = (double)x_pos;
  target_pose_msg.position.data[1] = (double)y_pos;
  target_pose_msg.position.data[2] = (double)z_pos;

  // Set timestamp for target_pose_msg
  unsigned long now_target_pose = micros();
  target_pose_msg.header.stamp.sec = now_target_pose / 1000000;
  target_pose_msg.header.stamp.nanosec = (now_target_pose % 1000000) * 1000;

  RCSOFTCHECK(rcl_publish(&target_pose_publisher, &target_pose_msg, NULL));

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(0)));
}