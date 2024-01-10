#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rclc_parameter/rclc_parameter.h>
#include <std_msgs/msg/bool.h>
#include <stdio.h>

static const uint8_t WING_RELAY_TRIGGER_PIN = 23;

bool enable_obstacle_notification = true;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_parameter_server_t parameter_server;
rcl_subscription_t subscriber;
std_msgs__msg__Bool msg;
rclc_executor_t executor;

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

void error_loop()
{
  while (1)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

bool parameter_callback(const Parameter* old_param, const Parameter* new_param,
                        void* context)
{
  if (old_param == NULL && new_param == NULL)
  {
    return false;
  }

  if (std::string(old_param->name.data) == "enable_obstacle_notification")
  {
    enable_obstacle_notification = new_param->value.bool_value;
    digitalWrite(WING_RELAY_TRIGGER_PIN, LOW);
  }

  return true;
}

void obstacle_detected_callback(const void* msgin)
{
  if (enable_obstacle_notification)
  {
    const auto* msg = static_cast<const std_msgs__msg__Bool*>(msgin);
    digitalWrite(WING_RELAY_TRIGGER_PIN, msg->data ? HIGH : LOW);
  }
}

void setup()
{
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(WING_RELAY_TRIGGER_PIN, OUTPUT);
  digitalWrite(WING_RELAY_TRIGGER_PIN, LOW);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "modot_uros", "", &support));

  // create parameter server
  RCCHECK(rclc_parameter_server_init_default(&parameter_server, &node));
  RCCHECK(rclc_add_parameter(&parameter_server, "enable_obstacle_notification",
                             RCLC_PARAMETER_BOOL));
  RCCHECK(rclc_parameter_set_bool(&parameter_server,
                                  "enable_obstacle_notification",
                                  enable_obstacle_notification));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "obstacle_detector/detected"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context,
                             RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + 1,
                             &allocator));
  RCCHECK(rclc_executor_add_parameter_server(&executor, &parameter_server,
                                             &parameter_callback));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &subscriber, &msg, &obstacle_detected_callback, ON_NEW_DATA));
}

void loop()
{
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
