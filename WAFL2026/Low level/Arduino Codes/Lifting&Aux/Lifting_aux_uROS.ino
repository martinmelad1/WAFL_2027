//*******************************************************************************//
//*     TITLE: Lifting Motor Control System with Limit Switches (micro-ROS)     *//
//*     AUTHOR: Omar Emad & Mohamed Montasser                                   *//
//*     DESCRIPTION: ROS2 subscriber-based motor control with local safety     *//
//*******************************************************************************//

#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h>

// =================================================
// PIN DEFINITIONS (MATCH STANDALONE)
// =================================================

// ---- Lifting motor ----
#define sig1        14
#define sig2        25
#define led         13
#define sw0         4
#define LimitUp     32
#define LimitDown   33

// ---- Lights (Active LOW relays) ----
#define RELAY_FRONT 17
#define RELAY_RIGHT 16
#define RELAY_LEFT  2
#define RELAY_BACK  15

int relays[4] = {
  RELAY_FRONT,
  RELAY_RIGHT,
  RELAY_LEFT,
  RELAY_BACK
};

// =================================================
// micro-ROS OBJECTS
// =================================================

rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

// Subscribers
rcl_subscription_t lift_sub;
rcl_subscription_t lights_sub;

// Messages
std_msgs__msg__Int32 lift_msg;
std_msgs__msg__Int32MultiArray lights_msg;

// =================================================
// COMMAND VARIABLES
// =================================================

// Lift motor command
volatile int lift_cmd = 0; // 0=STOP, 1=UP, 2=DOWN

// Lights command buffer
int light_state[4] = {0, 0, 0, 0};

// =================================================
// ROS CALLBACKS
// =================================================

void lift_callback(const void * msgin)
{
  const std_msgs__msg__Int32 * msg =
    (const std_msgs__msg__Int32 *)msgin;

  lift_cmd = msg->data;
}

void lights_callback(const void * msgin)
{
  const std_msgs__msg__Int32MultiArray * msg =
    (const std_msgs__msg__Int32MultiArray *)msgin;

  if (msg->data.size < 4) return;

  for (int i = 0; i < 4; i++) {
    light_state[i] = msg->data.data[i];
  }
}
int32_t lights_buffer[4];

// =================================================
// SETUP
// =================================================

void setup()
{
  // GPIO setup
  pinMode(sig1, OUTPUT);
  pinMode(sig2, OUTPUT);
  pinMode(led, OUTPUT);

  pinMode(sw0, INPUT_PULLUP);
  pinMode(LimitUp, INPUT_PULLUP);
  pinMode(LimitDown, INPUT_PULLUP);

  for (int i = 0; i < 4; i++) {
    pinMode(relays[i], OUTPUT);
    digitalWrite(relays[i], HIGH); // OFF (active LOW)
  }

  // micro-ROS over USB
  set_microros_wifi_transports(
    "EME-SIEMENS_GPS",
    "gps12345",  // <-- your Wi-Fi password
    "192.168.0.126",       // <-- your PC IP where micro-ROS agent runs
    8888                   // <-- port (keep same for all ESPs)
);

  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "lift_and_lights_node", "", &support);

  // Subscribers
  rclc_subscription_init_default(
    &lift_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/lift_cmd"
  );

  rclc_subscription_init_default(
    &lights_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "/lights_cmd"
  );

  // Executor
  rclc_executor_init(&executor, &support.context, 2, &allocator);
    
  // Allocate memory for Int32MultiArray
  lights_msg.data.data = lights_buffer;
  lights_msg.data.capacity = 4;
  lights_msg.data.size = 0;   // will be set on reception
  
  lights_msg.layout.dim.capacity = 0;
  lights_msg.layout.dim.size = 0;
  lights_msg.layout.dim.data = NULL;
  lights_msg.layout.data_offset = 0;

  rclc_executor_add_subscription(
    &executor, &lift_sub, &lift_msg, &lift_callback, ON_NEW_DATA
  );
  rclc_executor_add_subscription(
    &executor, &lights_sub, &lights_msg, &lights_callback, ON_NEW_DATA
  );
}

// =================================================
// LOOP
// =================================================

void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // =================================================
  // LIFT MOTOR LOGIC (UNCHANGED ALGORITHM)
// =================================================

  if (digitalRead(sw0) == 0) {
    digitalWrite(sig1, LOW);
    digitalWrite(sig2, LOW);
  }
  else if (digitalRead(LimitUp) == 0 && lift_cmd == 2) { // Down allowed
    digitalWrite(sig1, LOW);
    digitalWrite(sig2, HIGH);
  }
  else if (digitalRead(LimitDown) == 0 && lift_cmd == 1) { // Up allowed
    digitalWrite(sig1, HIGH);
    digitalWrite(sig2, LOW);
  }
  else if (digitalRead(LimitUp) == 0 || digitalRead(LimitDown) == 0) {
    digitalWrite(sig1, LOW);
    digitalWrite(sig2, LOW);
  }
  else if (lift_cmd == 1) { // UP
    digitalWrite(sig1, HIGH);
    digitalWrite(sig2, LOW);
  }
  else if (lift_cmd == 2) { // DOWN
    digitalWrite(sig1, LOW);
    digitalWrite(sig2, HIGH);
  }
  else {
    digitalWrite(sig1, LOW);
    digitalWrite(sig2, LOW);
  }

  // =================================================
  // LIGHTS CONTROL
  // =================================================

  for (int i = 0; i < 4; i++) {
    digitalWrite(relays[i], light_state[i] ? LOW : HIGH);
  }
}
