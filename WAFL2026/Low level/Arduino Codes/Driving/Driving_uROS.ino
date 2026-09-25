//*******************************************************************************//
//*     TITLE: Dual PWM velocity control with encoder feedback                  *//
//*     AUTHORS: Omar Emad & Mohamed Montasser                                  *//
//*     PWM VELOCITY VERSION                                                    *//
//*******************************************************************************//

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>

#include <WiFi.h>

//-------------------------------------------------------------------
// ROBOT CONSTANTS
//-------------------------------------------------------------------

const float WHEEL_RADIUS     = 0.1;
const int   PULSES_PER_REV   = 600;
float       GEAR_RATIO       = (1885.0 / 600.0);

//-------------------------------------------------------------------
// MOTOR PINS
//-------------------------------------------------------------------

const int M1_PWM = 32;
const int M1_DIR = 14;

const int M2_PWM = 22;
const int M2_DIR = 2;

//-------------------------------------------------------------------
// ENCODER PINS
//-------------------------------------------------------------------

const int ENCODER_A = 17;
const int ENCODER_B = 16;

//-------------------------------------------------------------------
// VARIABLES
//-------------------------------------------------------------------

// Used for odometry publishing
volatile long encoderOdom = 0;

// direction for odom sign
int direction = 1;

// RPM calculation
unsigned long lastTime  = 0;
long          lastCount = 0;

// Cached PWM values (signed: negative = reverse)
volatile int pwm_left_val  = 0;
volatile int pwm_right_val = 0;

//-------------------------------------------------------------------
// micro-ROS VARIABLES
//-------------------------------------------------------------------

// Two subscribers — one per wheel
rcl_subscription_t sub_left;
rcl_subscription_t sub_right;

std_msgs__msg__Float32 msg_left;
std_msgs__msg__Float32 msg_right;

// Encoder publisher (unchanged)
rcl_publisher_t        encoder_pub;
std_msgs__msg__Float32 encoder_msg;

rclc_support_t   support;
rcl_node_t       node;
rcl_allocator_t  allocator;
rclc_executor_t  executor;

//-------------------------------------------------------------------
// ENCODER ISR
//-------------------------------------------------------------------

void IRAM_ATTR encoderISR()
{
  encoderOdom++;
}

//-------------------------------------------------------------------
// MOTOR CONTROL
//-------------------------------------------------------------------

// Apply a signed PWM value [-255 … 255] to a single motor.
// channel 0 → M1,  channel 1 → M2
void applyMotor(int channel, int dirPin, int signedPWM)
{
  int clampedPWM = constrain(abs(signedPWM), 0, 255);

  if (signedPWM >= 0)
    digitalWrite(dirPin, LOW);   // forward
  else
    digitalWrite(dirPin, HIGH);  // backward

  ledcWrite(channel, clampedPWM);
}

//-------------------------------------------------------------------
// micro-ROS CALLBACKS
//-------------------------------------------------------------------

void left_pwm_callback(const void *msgin)
{
  const std_msgs__msg__Float32 *msg =
    (const std_msgs__msg__Float32 *)msgin;

  pwm_left_val = (int)msg->data;          // signed: negative = reverse
  applyMotor(0, M1_DIR, pwm_left_val);

  // Update odom direction based on left wheel as reference
  direction = (pwm_left_val >= 0) ? 1 : -1;
}

void right_pwm_callback(const void *msgin)
{
  const std_msgs__msg__Float32 *msg =
    (const std_msgs__msg__Float32 *)msgin;

  pwm_right_val = -(int)msg->data;         // signed: negative = reverse
  applyMotor(1, M2_DIR, pwm_right_val);
}

//-------------------------------------------------------------------
// RPM CALCULATION  (unchanged logic)
//-------------------------------------------------------------------

float calculateRPM()
{
  unsigned long currentTime = millis();
  unsigned long dt          = currentTime - lastTime;

  if (dt >= 200)
  {
    long diff  = encoderOdom - lastCount;
    lastCount  = encoderOdom;
    lastTime   = currentTime;

    float encoder_rps = (float)diff / PULSES_PER_REV;
    float wheel_rps   = encoder_rps / GEAR_RATIO;

    return wheel_rps * 60.0;
  }

  return -1;
}

//-------------------------------------------------------------------
// SETUP
//-------------------------------------------------------------------

void setup()
{
  Serial.begin(115200);
  delay(2000);
  Serial.println("Boot OK");

  set_microros_wifi_transports(
    "EME-SIEMENS_GPS",
    "gps12345",
    "192.168.0.126",
    8888
  );

  Serial.println("WiFi transport configured");
  Serial.print("ESP IP: ");
  Serial.println(WiFi.localIP());

  // PWM channels
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
  ledcAttachPin(M1_PWM, 0);
  ledcAttachPin(M2_PWM, 1);

  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_DIR, OUTPUT);

  // Encoder
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(
    digitalPinToInterrupt(ENCODER_B),
    encoderISR,
    RISING
  );

  delay(2000);

  // micro-ROS init
  allocator = rcl_get_default_allocator();

  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "robot_node", "", &support);

  // Subscriber — left wheel
  rclc_subscription_init_default(
    &sub_left,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/pwm_left"
  );

  // Subscriber — right wheel
  rclc_subscription_init_default(
    &sub_right,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/pwm_right"
  );

  // Publisher — encoder odometry (unchanged)
  rclc_publisher_init_default(
    &encoder_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/encoder_count"
  );

  // Executor needs 2 handles now (one per subscriber)
  rclc_executor_init(&executor, &support.context, 2, &allocator);

  rclc_executor_add_subscription(
    &executor, &sub_left,  &msg_left,
    &left_pwm_callback,  ON_NEW_DATA
  );

  rclc_executor_add_subscription(
    &executor, &sub_right, &msg_right,
    &right_pwm_callback, ON_NEW_DATA
  );

  lastTime  = millis();
  lastCount = 0;
}

//-------------------------------------------------------------------
// LOOP
//-------------------------------------------------------------------

void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // Continuous odom publishing (unchanged)
  encoder_msg.data = encoderOdom * direction;
  rcl_publish(&encoder_pub, &encoder_msg, NULL);

  // Optional RPM debug print
  float rpm = calculateRPM();
  if (rpm >= 0)
  {
    Serial.print("Encoder Odom = ");
    Serial.print(encoderOdom);
    Serial.print(" | Wheel RPM = ");
    Serial.println(rpm);
  }
}
