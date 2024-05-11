#include <Arduino.h>
#include <M5Unified.h>

#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

// ros2 definition =====================================================
#include <RingBuf.h>
#include <micro_ros_utilities/string_utilities.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rclc_parameter/rclc_parameter.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/string.h>
#include <std_srvs/srv/set_bool.h>

#include "w5500_m5_ethernet_transport.h"

// cybergear definitions ==============================================
#include <cybergear_can_interface_twai.hh>
#include <cybergear_controller.hh>
#include <cybergear_driver_defs.hh>
const uint8_t MASTER_CAN_ID = 0x00;

CybergearController controller = CybergearController(MASTER_CAN_ID);
CybergearCanInterfaceTwai can_interface;

static const uint8_t MOTOR_NUM = 2;
const std::vector<std::string> joint_names = {"mot1", "mot2"};
const std::vector<uint8_t> motor_ids = {0x7F, 0x7E};

struct BufData
{
  MotorStatus data[MOTOR_NUM];
};
QueueHandle_t motor_state_queue;

struct MotorCommand
{
  uint8_t id;
  uint8_t run_mode;
  float position = 0.0f;
  float velocity = 0.0f;
  float effort = 0.0f;
};
std::unordered_map<std::string, MotorCommand> motor_commands;

struct MotorParam
{
  uint8_t id;
  uint8_t run_mode = MODE_POSITION;
  float limit_torque = DEFAULT_TORQUE_LIMIT;
  float limit_speed = DEFAULT_VELOCITY_LIMIT;
  float limit_current = DEFAULT_CURRENT_LIMIT;
  float cur_kp = DEFAULT_CURRENT_KP;
  float cur_ki = DEFAULT_CURRENT_KI;
  float loc_kp = DEFAULT_POSITION_KP;
  float spd_kp = DEFAULT_VELOCITY_KP;
  float spd_ki = DEFAULT_VELOCITY_KI;
  float cur_filter_gain = DEFAULT_CURRENT_FINTER_GAIN;
};
std::unordered_map<std::string, MotorParam> motor_parameters;

// Network definitions ==========================================
byte local_mac[] = {0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED};
const IPAddress local_ip(192, 168, 50, 200);
const IPAddress host_ip(192, 168, 50, 76);
const uint16_t host_port = 2000;

// ROS definitions ==============================================
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// for joint_state pub
rcl_publisher_t publisher;
sensor_msgs__msg__JointState joint_state_msg;

// for joint_command sub
rcl_subscription_t subscription;
sensor_msgs__msg__JointState joint_command_msg;
rcl_timer_t timer;

// for time sync
rcl_timer_t sync_timer;

// for control_power status
rcl_service_t service;
std_srvs__srv__SetBool_Response res;
std_srvs__srv__SetBool_Request req;
bool motor_power_on_status = false;

// parameters
rclc_parameter_server_t param_server;

// for task control
void ros2_task(void * args);
void motor_control_task(void * args);
TaskHandle_t ros2_task_handle = NULL;
TaskHandle_t ros2_time_sync_task_handle = NULL;
TaskHandle_t motor_control_task_handle = NULL;

void update_motor_parameters()
{
  // check motor status
  if (motor_power_on_status) {
    return;
  }

  for (uint8_t idx = 0; idx < joint_names.size(); ++idx) {
    if (motor_parameters.find(joint_names[idx]) != motor_parameters.end()) {
      MotorParam param = motor_parameters[joint_names[idx]];
      controller.set_run_mode(param.id, param.run_mode);
      controller.set_speed_limit(param.id, param.limit_speed);
      controller.set_torque_limit(param.id, param.limit_torque);
      controller.set_current_limit(param.id, param.limit_current);
      controller.set_position_control_gain(param.id, param.loc_kp);
      controller.set_velocity_control_gain(param.id, param.spd_kp, param.spd_ki);
      controller.set_current_control_param(
        param.id, param.cur_kp, param.cur_ki, param.cur_filter_gain);

      // update motor command
      if (motor_commands.find(joint_names[idx]) != motor_commands.end()) {
        motor_commands[joint_names[idx]].run_mode = param.run_mode;
        motor_commands[joint_names[idx]].position = 0.0f;
        motor_commands[joint_names[idx]].velocity = 0.0f;
        motor_commands[joint_names[idx]].effort = 0.0f;
      }
    }
  }
}

void init_motor_parameters()
{
  for (uint8_t idx = 0; idx < joint_names.size(); ++idx) {
    MotorParam param;
    param.id = motor_ids[idx];
    motor_parameters[joint_names[idx]] = param;

    MotorCommand cmd;
    cmd.id = motor_ids[idx];
    cmd.run_mode = param.run_mode;
    motor_commands[joint_names[idx]] = cmd;
  }
}

void joint_command_callback(const void * msgin)
{
  const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;
  for (uint8_t idx = 0; idx < msg->name.size; ++idx) {
    // check joint name
    const std::string name = micro_ros_string_utilities_get_c_str(msg->name.data[idx]);
    if (motor_commands.find(name) == motor_commands.end()) {
      continue;
    }

    // set joint command
    if (msg->position.size > idx) {
      motor_commands[name].position = msg->position.data[idx];
    }
    if (msg->velocity.size > idx) {
      motor_commands[name].velocity = msg->velocity.data[idx];
    }
    if (msg->effort.size > idx) {
      motor_commands[name].effort = msg->effort.data[idx];
    }
  }
}

void joint_state_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  (void)last_call_time;
  (void)timer;

  BufData data;
  BaseType_t ret = xQueueReceive(motor_state_queue, &data, 0);
  if (ret != pdTRUE) return;

  joint_state_msg.header.stamp.sec = rmw_uros_epoch_millis() / 1000;
  joint_state_msg.header.stamp.nanosec = rmw_uros_epoch_nanos() % 1000000000UL;
  for (uint8_t idx = 0; idx < joint_names.size(); ++idx) {
    joint_state_msg.position.data[idx] = data.data[idx].position;
    joint_state_msg.velocity.data[idx] = data.data[idx].velocity;
    joint_state_msg.effort.data[idx] = data.data[idx].effort;
  }
  rcl_publish(&publisher, &joint_state_msg, NULL);
}

void ros2_sync_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  const int timeout_ms = 1000;
  rmw_uros_sync_session(timeout_ms);
}

void control_power_service_callback(const void * req, void * res)
{
  std_srvs__srv__SetBool_Request * req_in = (std_srvs__srv__SetBool_Request *)req;
  std_srvs__srv__SetBool_Response * res_out = (std_srvs__srv__SetBool_Response *)res;
  res_out->success = false;

  if (req_in->data) {
    // power on
    if (motor_power_on_status == false) {
      update_motor_parameters();
      controller.enable_motors();
      motor_power_on_status = true;
      res_out->success = true;
    }
  } else if (!req_in->data) {
    // power off
    if (motor_power_on_status == true) {
      controller.reset_motors();
      motor_power_on_status = false;
      res_out->success = true;
    }
  }
}

void init_ros_param_server()
{
  const rclc_parameter_options_t options = {
    .notify_changed_over_dds = true,
    .max_params = joint_names.size() * 10,
    .allow_undeclared_parameters = false,
    .low_mem_mode = false};
  rclc_parameter_server_init_with_option(&param_server, &node, &options);
  rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed);

  for (uint16_t idx = 0; idx < joint_names.size(); ++idx) {
    if (motor_parameters.find(joint_names[idx]) == motor_parameters.end()) continue;
    MotorParam param = motor_parameters[joint_names[idx]];

    {
      std::string param_name = joint_names[idx] + ".run_mode";
      rclc_add_parameter(&param_server, param_name.c_str(), RCLC_PARAMETER_INT);
      rclc_parameter_set_int(&param_server, param_name.c_str(), param.run_mode);
      rclc_add_parameter_constraint_integer(
        &param_server, param_name.c_str(), MODE_MOTION, MODE_CURRENT, 1);
    }

    {
      std::string param_name = joint_names[idx] + ".limit_torque";
      rclc_add_parameter(&param_server, param_name.c_str(), RCLC_PARAMETER_DOUBLE);
      rclc_parameter_set_double(&param_server, param_name.c_str(), param.limit_torque);
      rclc_add_parameter_constraint_double(
        &param_server, param_name.c_str(), LIMIT_TORQUE_MIN, LIMIT_TORQUE_MAX, 0.1);
    }

    {
      std::string param_name = joint_names[idx] + ".limit_speed";
      rclc_add_parameter(&param_server, param_name.c_str(), RCLC_PARAMETER_DOUBLE);
      rclc_parameter_set_double(&param_server, param_name.c_str(), param.limit_speed);
      rclc_add_parameter_constraint_double(
        &param_server, param_name.c_str(), LIMIT_SPD_MIN, LIMIT_SPD_MAX, 0.1);
    }

    {
      std::string param_name = joint_names[idx] + ".limit_current";
      rclc_add_parameter(&param_server, param_name.c_str(), RCLC_PARAMETER_DOUBLE);
      rclc_parameter_set_double(&param_server, param_name.c_str(), param.limit_current);
      rclc_add_parameter_constraint_double(&param_server, param_name.c_str(), 0.0, IQ_MAX, 0.1);
    }

    {
      std::string param_name = joint_names[idx] + ".cur_kp";
      rclc_add_parameter(&param_server, param_name.c_str(), RCLC_PARAMETER_DOUBLE);
      rclc_parameter_set_double(&param_server, param_name.c_str(), param.cur_kp);
      rclc_add_parameter_constraint_double(
        &param_server, param_name.c_str(), CUR_KP_MIN, 1.0, 0.001);
    }

    {
      std::string param_name = joint_names[idx] + ".cur_ki";
      rclc_add_parameter(&param_server, param_name.c_str(), RCLC_PARAMETER_DOUBLE);
      rclc_parameter_set_double(&param_server, param_name.c_str(), param.cur_ki);
      rclc_add_parameter_constraint_double(
        &param_server, param_name.c_str(), CUR_KI_MIN, 0.1, 0.0001);
    }

    {
      std::string param_name = joint_names[idx] + ".loc_kp";
      rclc_add_parameter(&param_server, param_name.c_str(), RCLC_PARAMETER_DOUBLE);
      rclc_parameter_set_double(&param_server, param_name.c_str(), param.loc_kp);
      rclc_add_parameter_constraint_double(
        &param_server, param_name.c_str(), LOC_KP_MIN, 100.0, 0.1);
    }

    {
      std::string param_name = joint_names[idx] + ".spd_kp";
      rclc_add_parameter(&param_server, param_name.c_str(), RCLC_PARAMETER_DOUBLE);
      rclc_parameter_set_double(&param_server, param_name.c_str(), param.spd_kp);
      rclc_add_parameter_constraint_double(&param_server, param_name.c_str(), 0.0, 50.0, 0.1);
    }

    {
      std::string param_name = joint_names[idx] + ".spd_ki";
      rclc_add_parameter(&param_server, param_name.c_str(), RCLC_PARAMETER_DOUBLE);
      rclc_parameter_set_double(&param_server, param_name.c_str(), param.spd_ki);
      rclc_add_parameter_constraint_double(&param_server, param_name.c_str(), 0.0, 0.05, 0.001);
    }

    {
      std::string param_name = joint_names[idx] + ".cur_filter_gain";
      rclc_add_parameter(&param_server, param_name.c_str(), RCLC_PARAMETER_DOUBLE);
      rclc_parameter_set_double(&param_server, param_name.c_str(), param.cur_filter_gain);
      rclc_add_parameter_constraint_double(&param_server, param_name.c_str(), 0.0, 1.0, 0.01);
    }
  }
}

void init_joint_state_msg(
  sensor_msgs__msg__JointState * msg, const std::vector<std::string> & names)
{
  rosidl_runtime_c__String__Sequence__init(&msg->name, names.size());
  for (uint8_t idx = 0; idx < names.size(); ++idx) {
    rosidl_runtime_c__String__assign(&msg->name.data[idx], names[idx].data());
  }

  rosidl_runtime_c__float64__Sequence__init(&msg->position, names.size());
  rosidl_runtime_c__float64__Sequence__init(&msg->velocity, names.size());
  rosidl_runtime_c__float64__Sequence__init(&msg->effort, names.size());
  for (uint8_t idx = 0; idx < names.size(); ++idx) {
    msg->position.data[idx] = 0.0;
    msg->velocity.data[idx] = 0.0;
    msg->effort.data[idx] = 0.0;
  }
}

std::vector<std::string> split(const std::string & str, const std::string & separator)
{
  if (separator == "") return {str};
  std::vector<std::string> result;
  std::string tstr = str + separator;
  long l = tstr.length(), sl = separator.length();
  std::string::size_type pos = 0, prev = 0;

  for (; pos < l && (pos = tstr.find(separator, pos)) != std::string::npos; prev = (pos += sl)) {
    result.emplace_back(tstr, prev, pos - prev);
  }
  return result;
}

bool on_parameter_changed(const Parameter * old_param, const Parameter * new_param, void * context)
{
  (void)context;

  if (new_param == NULL) {
    return false;
  }

  const std::string param_name(new_param->name.data);
  std::vector<std::string> param_names = split(param_name, ".");
  if (param_names.size() != 2) return false;
  if (motor_parameters.find(param_names[0]) == motor_parameters.end()) return false;

  if (param_names[1] == "run_mode") {
    motor_parameters[param_names[0]].run_mode = new_param->value.integer_value;

  } else if (param_names[1] == "limit_torque") {
    motor_parameters[param_names[0]].limit_torque =
      static_cast<float>(new_param->value.double_value);

  } else if (param_names[1] == "limit_speed") {
    motor_parameters[param_names[0]].limit_speed =
      static_cast<float>(new_param->value.double_value);

  } else if (param_names[1] == "limit_current") {
    motor_parameters[param_names[0]].limit_current =
      static_cast<float>(new_param->value.double_value);

  } else if (param_names[1] == "cur_kp") {
    motor_parameters[param_names[0]].cur_kp = static_cast<float>(new_param->value.double_value);

  } else if (param_names[1] == "cur_ki") {
    motor_parameters[param_names[0]].cur_ki = static_cast<float>(new_param->value.double_value);

  } else if (param_names[1] == "loc_kp") {
    motor_parameters[param_names[0]].loc_kp = static_cast<float>(new_param->value.double_value);

  } else if (param_names[1] == "spd_kp") {
    motor_parameters[param_names[0]].spd_kp = static_cast<float>(new_param->value.double_value);

  } else if (param_names[1] == "spd_ki") {
    motor_parameters[param_names[0]].spd_ki = static_cast<float>(new_param->value.double_value);

  } else if (param_names[1] == "cur_filter_gain") {
    motor_parameters[param_names[0]].cur_filter_gain =
      static_cast<float>(new_param->value.double_value);
  }

  return true;
}

// setup micro_ros_arduino ===============================================
void setup()
{
  // Display設定
  M5.Display.init();
  M5.Display.setRotation(3);
  M5.Display.setFont(&fonts::lgfxJapanGothicP_12);
  M5.Display.setTextColor(WHITE, BLACK);

  // Title表示
  M5.Display.setCursor(0, 0);
  M5.Display.println("ROS2 connection");

  motor_state_queue = xQueueCreate(1, sizeof(BufData));

  // Init communication interface
  set_microros_w5500_m5_ethernet_udp_transports(local_mac, local_ip, host_ip, host_port);

  allocator = rcl_get_default_allocator();

  // Create init_options
  M5.Display.println("init support ... ");
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  // rcl_init_options_set_domain_id(&init_options, 100);
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

  // Create node
  rclc_node_init_default(&node, "cgc_node", "cgc_node", &support);

  // Create executor
  // timer x 2
  // joint_state pub x 1
  // joint_command sub x 1
  // param server x RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(
    &executor, &support.context, 4 + RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + 4, &allocator);

  // Create // init msgs
  init_joint_state_msg(&joint_state_msg, joint_names);
  init_joint_state_msg(&joint_command_msg, joint_names);

  // pub sensor/joint_states
  rclc_publisher_init_default(
    &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "joint_state");

  // sub sensor/joint_states
  rclc_subscription_init_default(
    &subscription, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "joint_command");

  rclc_executor_add_subscription(
    &executor, &subscription, &joint_command_msg, &joint_command_callback, ON_NEW_DATA);

  // // setup timers
  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(2), joint_state_timer_callback);
  rclc_executor_add_timer(&executor, &timer);

  rclc_timer_init_default(&sync_timer, &support, RCL_MS_TO_NS(2000), ros2_sync_timer_callback);
  rclc_executor_add_timer(&executor, &sync_timer);

  // Init parameter server
  init_motor_parameters();
  init_ros_param_server();

  // motor control services
  rclc_service_init_default(
    &service, &node, ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool), "control_power");
  rclc_executor_add_service(&executor, &service, &req, &res, control_power_service_callback);

  // init cybergear interfaces
  can_interface.init(1, 2);
  controller.init(motor_ids, MODE_CURRENT, &can_interface, 250);

  M5.Display.println("end init");

  // create task
  xTaskCreatePinnedToCore(ros2_task, "ros2_task", 1024 * 4, NULL, 1, &ros2_task_handle, 0);
  xTaskCreatePinnedToCore(
    motor_control_task, "motor_control_task", 1024 * 4, NULL, 1, &motor_control_task_handle, 1);
}

void ros2_task(void * args)
{
  while (true) {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  }
}

void motor_control_task(void * args)
{
  std::vector<float> currents(motor_ids.size(), 0.0f);
  std::vector<MotorStatus> motor_status;
  BufData data;

  while (true) {
    // process motor control commands
    if (motor_power_on_status == false) {
      // send dummy control command to get motor status
      controller.send_current_command(motor_ids, currents);

    } else {
      for (uint8_t idx = 0; idx < joint_names.size(); ++idx) {
        // control each motor
        MotorCommand cmd = motor_commands[joint_names[idx]];

        if (cmd.run_mode == MODE_POSITION) {
          controller.send_position_command(cmd.id, cmd.position);

        } else if (cmd.run_mode == MODE_SPEED) {
          controller.send_speed_command(cmd.id, cmd.velocity);

        } else if (cmd.run_mode == MODE_CURRENT) {
          controller.send_current_command(cmd.id, cmd.effort);

        } else if (cmd.run_mode == MODE_MOTION) {
          // NOT SUPPORTED
        }
      }
    }

    // Send motor status to other process
    if (controller.process_packet()) {
      controller.get_motor_status(motor_status);
      if (motor_status.size() == joint_names.size()) {
        for (uint8_t idx = 0; idx < motor_status.size(); ++idx) data.data[idx] = motor_status[idx];
        xQueueOverwrite(motor_state_queue, &data);
      }
    }
  }
}

// loop micro_ros_arduino ===============================================
void loop()
{
  M5.update();
  delay(100);
}
