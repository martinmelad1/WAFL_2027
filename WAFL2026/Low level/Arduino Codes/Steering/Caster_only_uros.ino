/*
===============================================================
  MICRO-ROS ENCODER-DRIVEN DC MOTOR POSITION CONTROL
---------------------------------------------------------------
  • Subscribes to /target_angle  (std_msgs/Float32)
  • Publishes  to /actual_angle  (std_msgs/Float32)
  • Full quadrature decoding on both channels (CHANGE interrupt)
    → immune to missed edges caused by ESP32 WiFi IRQ masking
  • Atomic encoder read to prevent race conditions

  ROS2 Topics:
    Subscribe : /target_angle  [std_msgs/msg/Float32]
    Publish   : /actual_angle  [std_msgs/msg/Float32]

  Author: Mohamed Montasser & Omar Emad
===============================================================
*/

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

// ========================
// USER SETTINGS
// ========================
const float PPR = 23700.0;

const int pinA    = 2;   // Encoder A (interrupt)
const int pinB    = 15;    // Encoder B (interrupt)
const int motorPWM = 13;
const int motorDIR = 14;

// ========================
// ENCODER STATE
// ========================
// Full quadrature state machine — tracks all 4 transitions per cycle.
// This makes every count bidirectional and correct even when WiFi
// briefly masks an interrupt, because we always look at BOTH pins.
volatile long encoderCount = 0;

// Previous 2-bit state of {A, B}
volatile uint8_t lastEncState = 0;

// Quadrature truth table:
// Maps (prevState << 2 | currState) → {-1, 0, +1}
// Only the 8 valid transitions are non-zero.
static const int8_t QEM[16] = {
//  curr: 00  01  10  11
         0,  -1,  1,  0,   // prev 00
         1,   0,  0, -1,   // prev 01
        -1,   0,  0,  1,   // prev 10
         0,   1, -1,  0    // prev 11
};

// ========================
// MOTION STATE
// ========================
static bool  moving       = false;
static long  targetPulse  = 0;
static bool  directionSet = false;

// ========================
// MICRO-ROS OBJECTS
// ========================
rcl_publisher_t    actual_angle_publisher;
rcl_subscription_t target_angle_subscriber;

std_msgs__msg__Float32 actual_angle_msg;
std_msgs__msg__Float32 target_angle_msg;

rclc_executor_t executor;
rclc_support_t  support;
rcl_allocator_t allocator;
rcl_node_t      node;
rcl_timer_t     timer;

#define TIMER_TIMEOUT_MS 50

#define RCCHECK(fn)     { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

// ========================
// HELPERS
// ========================
void error_loop() { while (true) { delay(100); } }

// Atomic read of a volatile long on ESP32 (no 64-bit issue here,
// but disabling interrupts guarantees we never read a half-updated value).
long readEncoderAtomic() {
  portDISABLE_INTERRUPTS();
  long val = encoderCount;
  portENABLE_INTERRUPTS();
  return val;
}

// ========================
// FULL-QUADRATURE ISR
// ========================
// Attached to CHANGE on BOTH pinA and pinB.
// The QEM table decides direction from the 2-bit state transition,
// so a missed WiFi-induced edge can only produce a zero step (no count),
// not a phantom count in the wrong direction.
void IRAM_ATTR encoder_ISR() {
  uint8_t a = digitalRead(pinA);
  uint8_t b = digitalRead(pinB);
  uint8_t currState = (a << 1) | b;
  uint8_t idx = (lastEncState << 2) | currState;
  encoderCount += QEM[idx];
  lastEncState = currState;
}

// ========================
// MOTOR CONTROL
// ========================
void motorForward(int speed)  { digitalWrite(motorDIR, HIGH); analogWrite(motorPWM, speed); }
void motorBackward(int speed) { digitalWrite(motorDIR, LOW);  analogWrite(motorPWM, speed); }
void motorStop()              { analogWrite(motorPWM, 0); }

// ========================
// SUBSCRIBER CALLBACK
// ========================
void target_angle_callback(const void* msgin) {
  const std_msgs__msg__Float32* msg = (const std_msgs__msg__Float32*)msgin;
  float angle = msg->data;
  if (angle == 0.0f) return;

  targetPulse  = (long)(angle / 360.0f * PPR);
  moving       = true;
  directionSet = false;
}

// ========================
// TIMER CALLBACK — Publish feedback
// ========================
void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    long  pulses       = readEncoderAtomic();
    float currentAngle = (pulses * 360.0f) / PPR;
    actual_angle_msg.data = currentAngle;
    RCSOFTCHECK(rcl_publish(&actual_angle_publisher, &actual_angle_msg, NULL));
  }
}

// ========================
// SETUP
// ========================
void setup() {
  // Encoder pins
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);

  // Read initial state so first ISR transition is correct
  lastEncState = ((digitalRead(pinA) << 1) | digitalRead(pinB));

  // Attach BOTH pins to the SAME ISR on CHANGE (full quadrature)
  attachInterrupt(digitalPinToInterrupt(pinA), encoder_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB), encoder_ISR, CHANGE);

  // Motor pins
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDIR, OUTPUT);
  motorStop();

  // Micro-ROS WiFi transport
  set_microros_wifi_transports(
    "EME-SIEMENS_GPS",
    "gps12345",
    "192.168.0.126",
    8888
  );

  delay(2000);

  // Micro-ROS init
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "motor_position_node", "", &support));

  // Publisher
  RCCHECK(rclc_publisher_init_default(
    &actual_angle_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "actual_angle"
  ));

  // Subscriber
  RCCHECK(rclc_subscription_init_default(
    &target_angle_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "target_angle"
  ));

  // Timer (50 ms feedback publish)
  RCCHECK(rclc_timer_init_default(
    &timer, &support,
    RCL_MS_TO_NS(TIMER_TIMEOUT_MS),
    timer_callback
  ));

  // Executor: 2 handles
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &target_angle_subscriber,
    &target_angle_msg, &target_angle_callback,
    ON_NEW_DATA
  ));
}

// ========================
// MAIN LOOP
// ========================
void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  if (moving) {
    long current = readEncoderAtomic();   // atomic read everywhere
    long error   = targetPulse - current;

    // Set direction once at start of motion
    if (!directionSet) {
      if (error > 0) motorForward(120);
      else           motorBackward(120);
      directionSet = true;
    }

    // Normal stop — within deadband
    if (abs(error) < 80) {
      motorStop();
      moving = false;
      return;
    }

    // Overshoot protection
    if (error < 0 && digitalRead(motorDIR) == HIGH) {
      motorStop();
      moving = false;
    }
    if (error > 0 && digitalRead(motorDIR) == LOW) {
      motorStop();
      moving = false;
    }
  }
}
