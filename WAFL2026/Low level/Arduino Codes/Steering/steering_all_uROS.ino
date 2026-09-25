//*********************************************************************************//
//*     TITLE: Closed-Loop Steering Control with Vibration Compensation          *//
//*     AUTHORS: Omar Emad & Mohamed Montasser                                    *//
//*     DATE: 27/JAN/2026                                                         *//
//*     DESCRIPTION:                                                              *//
//*     Stepper steering with encoder feedback to auto-correct vibration drift.  *//
//*********************************************************************************//

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

// =================================================
// PIN CONFIGURATION
// =================================================

// ---- Stepper (TB6600) ----
#define STEP_PIN   27
#define DIR_PIN    32
#define ENA_PIN    33

// ---- Steering Encoder (NEW – 600 PPR) ----
#define STEER_ENC_A 17
#define STEER_ENC_B 16

// ---- DC Motor + Encoder (UNCHANGED) ----
#define DC_ENC_A   15
#define DC_ENC_B   2
#define MOTOR_PWM  13
#define MOTOR_DIR  14

// =================================================
// CONSTANTS
// =================================================

#define STEPPER_PPR 8250
#define STEER_ENCODER_PPR 600
#define STEERING_DEADBAND 4     //  NEW (anti-vibration zone)

unsigned int step_delay_us = 800;
const float DC_PPR = 23700.0;

// =================================================
// micro-ROS objects
// =================================================

rcl_node_t node;
rcl_subscription_t subscriber;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

std_msgs__msg__Float32 angle_msg;

// =================================================
// STEERING VARIABLES
// =================================================

// Encoder feedback
volatile long steerEncoderCount = 0;

// Target reference (PERSISTENT)
long steer_target_ticks = 0;
bool steering_initialized = false;

// =================================================
// DC MOTOR VARIABLES (UNCHANGED)
// =================================================

volatile long dcEncoderCount = 0;
long dc_targetPulse = 0;
bool dc_moving = false;
bool dc_directionSet = false;

// =================================================
// STEERING ENCODER ISR  NEW
// =================================================

void IRAM_ATTR steeringEncoderISR() {
  int b = digitalRead(STEER_ENC_B);
  if (b == HIGH) steerEncoderCount++;
  else steerEncoderCount--;
}

// =================================================
// DC MOTOR ENCODER ISR (UNCHANGED)
// =================================================

void IRAM_ATTR dcEncoderISR() {
  int b = digitalRead(DC_ENC_B);
  if (b == HIGH) dcEncoderCount++;
  else dcEncoderCount--;
}

// =================================================
// ROS CALLBACK (updates TARGET ONLY)
// =================================================

void angle_callback(const void *msgin)
{
  const std_msgs__msg__Float32 *msg =
    (const std_msgs__msg__Float32 *)msgin;

  float angle = msg->data;
  if (angle == 0) return;

  steer_target_ticks =
    (long)((angle / 360.0) * STEER_ENCODER_PPR);

  steering_initialized = true;   // latch reference
}

// =================================================
// STEP ONCE (MICRO-CORRECTION)
// =================================================

void stepOnce(bool direction) {
  digitalWrite(ENA_PIN, LOW);
  digitalWrite(DIR_PIN, direction ? HIGH : LOW);

  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(step_delay_us);

  digitalWrite(ENA_PIN, HIGH);
}

// =================================================
// SETUP
// =================================================

void setup()
{
  Serial.begin(115200);

  // Stepper
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, HIGH);

  // Steering Encoder for Correction
  pinMode(STEER_ENC_A, INPUT_PULLUP);
  pinMode(STEER_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(STEER_ENC_A),
                  steeringEncoderISR, RISING);

  // DC Motor (UNCHANGED)
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(DC_ENC_A, INPUT_PULLUP);
  pinMode(DC_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(DC_ENC_A),
                  dcEncoderISR, RISING);

  // micro-ROS WiFi
  set_microros_wifi_transports(
    "EME-SIEMENS_GPS",
    "gps12345",
    "192.168.0.107",
    8888
  );

  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "steering_node", "", &support);

  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/target_angle"
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(
    &executor, &subscriber, &angle_msg,
    &angle_callback, ON_NEW_DATA
  );
}

// =================================================
// LOOP
// =================================================

void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

  // =================================================
  // CLOSED-LOOP STEERING HOLD 
  // =================================================

  if (steering_initialized) {
    long error = steer_target_ticks - steerEncoderCount;

    if (abs(error) > STEERING_DEADBAND) {
      if (error > 0)
        stepOnce(true);   // correct right
      else
        stepOnce(false);  // correct left
    }
  }

  // =================================================
  // DC MOTOR CONTROL (UNCHANGED LOGIC)
  // =================================================

  if (dc_moving) {
    long error = dc_targetPulse - dcEncoderCount;

    if (!dc_directionSet) {
      digitalWrite(MOTOR_DIR, error > 0 ? HIGH : LOW);
      analogWrite(MOTOR_PWM, 120);
      dc_directionSet = true;
    }

    if (abs(error) < 80) {
      analogWrite(MOTOR_PWM, 0);
      dc_moving = false;
    }
  }
}
