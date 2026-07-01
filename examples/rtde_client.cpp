// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2020 FZI Forschungszentrum Informatik
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner mauch@fzi.de
 * \date    2020-08-12
 *
 */
//----------------------------------------------------------------------

#include <ur_client_library/rtde/rtde_client.h>
#include <ur_client_library/helpers.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <fstream>
#include <memory>
#include <vector>
#include <cstdint>
 
#ifdef _WIN32
#include <windows.h>
#endif
 
using namespace urcl;
 
const std::string DEFAULT_ROBOT_IP = "10.54.5.169";
const std::string OUTPUT_RECIPE = "C:/Users/MIRserv/Desktop/ur_client_library_felix/Universal_Robots_Client_Library/examples/resources/rtde_output_recipe.txt";
const std::string INPUT_RECIPE = "C:/Users/MIRserv/Desktop/ur_client_library_felix/Universal_Robots_Client_Library/examples/resources/rtde_input_recipe.txt";
 

const std::string KEY_TIMESTAMP = "timestamp";
const std::string KEY_TARGET_Q = "target_q";
const std::string KEY_TARGET_QD = "target_qd";
const std::string KEY_TARGET_QDD = "target_qdd";
const std::string KEY_TARGET_CURRENT = "target_current";
const std::string KEY_TARGET_MOMENT = "target_moment";
const std::string KEY_ACTUAL_Q = "actual_q";
const std::string KEY_ACTUAL_QD = "actual_qd";
const std::string KEY_ACTUAL_CURRENT = "actual_current";
const std::string KEY_ACTUAL_CURRENT_WINDOW = "actual_current_window";
const std::string KEY_ACTUAL_CURRENT_AS_TORQUE = "actual_current_as_torque";
const std::string KEY_JOINT_CONTROL_OUTPUT = "joint_control_output";
const std::string KEY_ACTUAL_TCP_POSE = "actual_TCP_pose";
const std::string KEY_ACTUAL_TCP_SPEED = "actual_TCP_speed";
const std::string KEY_ACTUAL_TCP_FORCE = "actual_TCP_force";
const std::string KEY_TARGET_TCP_POSE = "target_TCP_pose";
const std::string KEY_TARGET_TCP_SPEED = "target_TCP_speed";
const std::string KEY_TCP_OFFSET = "tcp_offset";
const std::string KEY_ACTUAL_TCP_ACCELERATION = "actual_TCP_acceleration";
const std::string KEY_TARGET_TCP_ACCELERATION = "target_TCP_acceleration";
const std::string KEY_ACTUAL_DIGITAL_INPUT_BITS = "actual_digital_input_bits";
const std::string KEY_ACTUAL_CONFIG_DIGITAL_INPUT_BITS = "actual_configurable_digital_input_bits";
const std::string KEY_JOINT_TEMPERATURES = "joint_temperatures";
const std::string KEY_ACTUAL_EXECUTION_TIME = "actual_execution_time";
const std::string KEY_TARGET_EXECUTION_TIME = "target_execution_time";
const std::string KEY_ROBOT_MODE = "robot_mode";
const std::string KEY_JOINT_MODE = "joint_mode";
const std::string KEY_SAFETY_MODE = "safety_mode";
const std::string KEY_SAFETY_STATUS = "safety_status";
const std::string KEY_ACTUAL_TOOL_ACCELEROMETER = "actual_tool_accelerometer";
const std::string KEY_SPEED_SCALING = "speed_scaling";
const std::string KEY_TARGET_SPEED_FRACTION = "target_speed_fraction";
const std::string KEY_ACTUAL_MOMENTUM = "actual_momentum";
const std::string KEY_ACTUAL_MAIN_VOLTAGE = "actual_main_voltage";
const std::string KEY_ACTUAL_ROBOT_VOLTAGE = "actual_robot_voltage";
const std::string KEY_ACTUAL_ROBOT_CURRENT = "actual_robot_current";
const std::string KEY_ACTUAL_JOINT_VOLTAGE = "actual_joint_voltage";
const std::string KEY_ACTUAL_DIGITAL_OUTPUT_BITS = "actual_digital_output_bits";
const std::string KEY_ACTUAL_CONFIG_DIGITAL_OUTPUT_BITS = "actual_configurable_digital_output_bits";
const std::string KEY_RUNTIME_STATE = "runtime_state";
const std::string KEY_ELBOW_POSITION = "elbow_position";
const std::string KEY_ELBOW_VELOCITY = "elbow_velocity";
const std::string KEY_ROBOT_STATUS_BITS = "robot_status_bits";
const std::string KEY_SAFETY_STATUS_BITS = "safety_status_bits";
const std::string KEY_ANALOG_IO_TYPES = "analog_io_types";
const std::string KEY_STANDARD_ANALOG_INPUT0 = "standard_analog_input0";
const std::string KEY_STANDARD_ANALOG_INPUT1 = "standard_analog_input1";
const std::string KEY_STANDARD_ANALOG_OUTPUT0 = "standard_analog_output0";
const std::string KEY_STANDARD_ANALOG_OUTPUT1 = "standard_analog_output1";
const std::string KEY_IO_CURRENT = "io_current";
const std::string KEY_INPUT_BIT_REG_0_31 = "input_bit_registers0_to_31";
const std::string KEY_INPUT_BIT_REG_32_63 = "input_bit_registers32_to_63";
const std::string KEY_OUTPUT_BIT_REG_0_31 = "output_bit_registers0_to_31";
const std::string KEY_OUTPUT_BIT_REG_32_63 = "output_bit_registers32_to_63";
const std::string KEY_ACTUAL_ROBOT_ENERGY_CONSUMED = "actual_robot_energy_consumed";
const std::string KEY_ACTUAL_ROBOT_BRAKING_ENERGY = "actual_robot_braking_energy_dissipated";
const std::string KEY_ENCODER0_RAW = "encoder0_raw";
const std::string KEY_ENCODER1_RAW = "encoder1_raw";
const std::string KEY_EUROMAP67_INPUT_BITS = "euromap67_input_bits";
const std::string KEY_EUROMAP67_OUTPUT_BITS = "euromap67_output_bits";
const std::string KEY_EUROMAP67_24V_VOLTAGE = "euromap67_24V_voltage";
const std::string KEY_EUROMAP67_24V_CURRENT = "euromap67_24V_current";
const std::string KEY_TOOL_MODE = "tool_mode";
const std::string KEY_TOOL_ANALOG_INPUT_TYPES = "tool_analog_input_types";
const std::string KEY_TOOL_ANALOG_INPUT0 = "tool_analog_input0";
const std::string KEY_TOOL_ANALOG_INPUT1 = "tool_analog_input1";
const std::string KEY_TOOL_OUTPUT_VOLTAGE = "tool_output_voltage";
const std::string KEY_TOOL_OUTPUT_CURRENT = "tool_output_current";
const std::string KEY_TOOL_TEMPERATURE = "tool_temperature";
const std::string KEY_TOOL_OUTPUT_MODE = "tool_output_mode";
const std::string KEY_TOOL_DIGITAL_OUTPUT0_MODE = "tool_digital_output0_mode";
const std::string KEY_TOOL_DIGITAL_OUTPUT1_MODE = "tool_digital_output1_mode";
const std::string KEY_TCP_FORCE_SCALAR = "tcp_force_scalar";
const std::string KEY_JOINT_POS_DEV_RATIO = "joint_position_deviation_ratio";
const std::string KEY_COLLISION_DETECT_RATIO = "collision_detection_ratio";
const std::string KEY_FT_RAW_WRENCH = "ft_raw_wrench";
const std::string KEY_WRENCH_CALC_FROM_CURRENTS = "wrench_calc_from_currents";
const std::string KEY_PAYLOAD = "payload";
const std::string KEY_PAYLOAD_COG = "payload_cog";
const std::string KEY_PAYLOAD_INERTIA = "payload_inertia";
const std::string KEY_SCRIPT_CONTROL_LINE = "script_control_line";
const std::string KEY_TIME_SCALE_SOURCE = "time_scale_source";
const std::string KEY_TARGET_GRAVITY = "target_gravity";
const std::string KEY_TARGET_BASE_ACCEL = "target_base_acceleration";
// const std::string KEY_CONTROLLER_STEP = "controller_step";
 
 
std::thread g_writer_thread;
moodycamel::ReaderWriterQueue<long long> g_durations_queue{ 1024 };
bool g_running = false;
 
void writerThreadFunc(const std::string& filename, const std::string& column_header = "cycle_time_microseconds")
{
  std::ofstream out_file(filename, std::ios::out);
  out_file << column_header << "\n";
  while (g_running || g_durations_queue.peek() != nullptr)
  {
    long long duration;
    while (g_durations_queue.tryDequeue(duration))
    {
      out_file << duration << "\n";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  out_file.close();
}
 
int main(int argc, char* argv[])
{

  pprocess_t process = pprocess_self();
  pthread_t thread = pthread_self();

#ifdef _WIN32
  // Assign logical CPUs 6 and 7 to this process
  DWORD_PTR process_mask = (1ULL << 6) | (1ULL << 7);
  if (!setProcessAffinity(process, process_mask))
  {
    URCL_LOG_ERROR("Failed to set process affinity");
  }

  // Assign logical CPU 7 to this thread
  DWORD_PTR thread_mask = (1ULL << 7);
  if (!setThreadAffinity(thread, thread_mask))
  {
    URCL_LOG_ERROR("Failed to set thread affinity");
  }
#else
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);

  // Assign logical CPU 7 to this thread
  CPU_SET(7, &cpuset);

  if (!setThreadAffinity(thread, cpuset))
  {
    URCL_LOG_ERROR("Failed to set thread affinity");
  }
#endif

  // Set thread and process to maximum priority
  const int max_prio = sched_get_priority_max(SCHED_FIFO);
  if (!setFiFoScheduling(thread, max_prio))
  {
    URCL_LOG_ERROR("Failed to set FIFO scheduling");
  }

  std::string robot_ip = DEFAULT_ROBOT_IP;
  if (argc > 1) { robot_ip = std::string(argv[1]); }
  int second_to_run = -1;
  if (argc > 2) { second_to_run = std::stoi(argv[2]); }
  g_running = true;
  g_writer_thread = std::thread(&writerThreadFunc, robot_ip + "_cycle_times.csv", "cycle_time_microseconds");
 
  comm::INotifier notifier;
  const double rtde_frequency = 500;  
  rtde_interface::RTDEClient my_client(robot_ip, notifier, OUTPUT_RECIPE, INPUT_RECIPE, rtde_frequency);
  my_client.init();
 
  double timestamp = 0.0;
  urcl::vector6d_t target_q, target_qd, target_qdd, target_current, target_moment;
  urcl::vector6d_t actual_q, actual_qd, actual_current, actual_current_window, actual_current_as_torque, joint_control_output;
  urcl::vector6d_t actual_TCP_pose, actual_TCP_speed, actual_TCP_force, target_TCP_pose, target_TCP_speed;
  urcl::vector6d_t tcp_offset, actual_TCP_acceleration, target_TCP_acceleration;
  uint64_t actual_digital_input_bits = 0, actual_configurable_digital_input_bits = 0;
  urcl::vector6d_t joint_temperatures;
  double actual_execution_time = 0.0, target_execution_time = 0.0;
  int32_t robot_mode = 0;
  std::array<int32_t, 6> joint_mode;
  int32_t safety_mode = 0, safety_status = 0;
  urcl::vector3d_t actual_tool_accelerometer;
  double speed_scaling = 0.0, actual_momentum = 0.0;
  double actual_main_voltage = 0.0, actual_robot_voltage = 0.0, actual_robot_current = 0.0;
  urcl::vector6d_t actual_joint_voltage;
  uint64_t actual_digital_output_bits = 0, actual_configurable_digital_output_bits = 0;
  uint32_t runtime_state = 0;
  urcl::vector3d_t elbow_position, elbow_velocity;
  uint32_t robot_status_bits = 0, safety_status_bits = 0, analog_io_types = 0;
  double standard_analog_input0 = 0.0, standard_analog_input1 = 0.0, standard_analog_output0 = 0.0, standard_analog_output1 = 0.0, io_current = 0.0;
  uint32_t input_bit_reg_0_31 = 0, input_bit_reg_32_63 = 0, output_bit_reg_0_31 = 0, output_bit_reg_32_63 = 0;
  bool input_bit_register[64]; 
  int32_t input_int_register[48];
  double input_double_register[48]; 
  bool output_bit_register[64]; 
  int32_t output_int_register[48]; 
  double output_double_register[48]; 
  double actual_robot_energy_consumed = 0.0, actual_robot_braking_energy_dissipated = 0.0;
  int32_t encoder0_raw = 0, encoder1_raw = 0;
  uint32_t euromap67_input_bits = 0, euromap67_output_bits = 0;
  double euromap67_24V_voltage = 0.0, euromap67_24V_current = 0.0;
  uint32_t tool_mode = 0, tool_analog_input_types = 0;
  double tool_analog_input0 = 0.0, tool_analog_input1 = 0.0;
  int32_t tool_output_voltage = 0;
  double tool_output_current = 0.0, tool_temperature = 0.0;
  uint8_t tool_output_mode = 0, tool_digital_output0_mode = 0, tool_digital_output1_mode = 0;
  double tcp_force_scalar = 0.0, joint_position_deviation_ratio = 0.0, collision_detection_ratio = 0.0;
  urcl::vector6d_t ft_raw_wrench, wrench_calc_from_currents;
  double payload = 0.0;
  urcl::vector3d_t payload_cog;
  urcl::vector6d_t payload_inertia;
  uint32_t script_control_line = 0;
  int32_t time_scale_source = 0;
  urcl::vector3d_t target_gravity;
  urcl::vector6d_t target_base_acceleration;
  // uint64_t controller_step = 0;

  double speed_slider_increment = 0.01;
  double speed_slider_fraction = 1.0;
  double target_speed_fraction = 1.0;
  double analog_val = 0.0;
  double analog_increment = 0.01;
  int int_register = 0;
  double double_register = 0.0;
  double force_val = 0.0;
  double force_increment = 0.5;
  uint8_t digital_values = 0;
  uint8_t digital_mask = 0x03;  
 
  auto data_pkg = std::make_unique<rtde_interface::DataPackage>(my_client.getOutputRecipe());
  my_client.start(false);
  auto start_time = std::chrono::high_resolution_clock::now();
  auto cycle_start = start_time;
  auto cycle_end = start_time;
  int loop_counter = 0;
 
  while (second_to_run <= 0 || std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start_time).count() < second_to_run)
  {
    cycle_start = std::chrono::high_resolution_clock::now();
    bool success = my_client.getDataPackageBlocking(data_pkg);
    if (success)
    {
      // ==============================================================================
      // EXTRACCIÓN DE DATOS (GET DATA) DE TODAS LAS VARIABLES
      // ==============================================================================
      data_pkg->getData(KEY_TIMESTAMP, timestamp);
      data_pkg->getData(KEY_TARGET_Q, target_q);
      data_pkg->getData(KEY_TARGET_QD, target_qd);
      data_pkg->getData(KEY_TARGET_QDD, target_qdd);
      data_pkg->getData(KEY_TARGET_CURRENT, target_current);
      data_pkg->getData(KEY_TARGET_MOMENT, target_moment);
      data_pkg->getData(KEY_ACTUAL_Q, actual_q);
      data_pkg->getData(KEY_ACTUAL_QD, actual_qd);
      data_pkg->getData(KEY_ACTUAL_CURRENT, actual_current);
      data_pkg->getData(KEY_ACTUAL_CURRENT_WINDOW, actual_current_window);
      data_pkg->getData(KEY_ACTUAL_CURRENT_AS_TORQUE, actual_current_as_torque);
      data_pkg->getData(KEY_JOINT_CONTROL_OUTPUT, joint_control_output);
      data_pkg->getData(KEY_ACTUAL_TCP_POSE, actual_TCP_pose);
      data_pkg->getData(KEY_ACTUAL_TCP_SPEED, actual_TCP_speed);
      data_pkg->getData(KEY_ACTUAL_TCP_FORCE, actual_TCP_force);
      data_pkg->getData(KEY_TARGET_TCP_POSE, target_TCP_pose);
      data_pkg->getData(KEY_TARGET_TCP_SPEED, target_TCP_speed);
      data_pkg->getData(KEY_TCP_OFFSET, tcp_offset);
      data_pkg->getData(KEY_ACTUAL_TCP_ACCELERATION, actual_TCP_acceleration);
      data_pkg->getData(KEY_TARGET_TCP_ACCELERATION, target_TCP_acceleration);
      data_pkg->getData(KEY_ACTUAL_DIGITAL_INPUT_BITS, actual_digital_input_bits);
      data_pkg->getData(KEY_ACTUAL_CONFIG_DIGITAL_INPUT_BITS, actual_configurable_digital_input_bits);
      data_pkg->getData(KEY_JOINT_TEMPERATURES, joint_temperatures);
      data_pkg->getData(KEY_ACTUAL_EXECUTION_TIME, actual_execution_time);
      data_pkg->getData(KEY_TARGET_EXECUTION_TIME, target_execution_time);
      data_pkg->getData(KEY_ROBOT_MODE, robot_mode);
      data_pkg->getData(KEY_SAFETY_MODE, safety_mode);
      data_pkg->getData(KEY_SAFETY_STATUS, safety_status);
      data_pkg->getData(KEY_ACTUAL_TOOL_ACCELEROMETER, actual_tool_accelerometer);
      data_pkg->getData(KEY_SPEED_SCALING, speed_scaling);
      data_pkg->getData(KEY_TARGET_SPEED_FRACTION, target_speed_fraction);
      data_pkg->getData(KEY_ACTUAL_MOMENTUM, actual_momentum);
      data_pkg->getData(KEY_ACTUAL_MAIN_VOLTAGE, actual_main_voltage);
      data_pkg->getData(KEY_ACTUAL_ROBOT_VOLTAGE, actual_robot_voltage);
      data_pkg->getData(KEY_ACTUAL_ROBOT_CURRENT, actual_robot_current);
      data_pkg->getData(KEY_ACTUAL_JOINT_VOLTAGE, actual_joint_voltage);
      data_pkg->getData(KEY_ACTUAL_DIGITAL_OUTPUT_BITS, actual_digital_output_bits);
      data_pkg->getData(KEY_ACTUAL_CONFIG_DIGITAL_OUTPUT_BITS, actual_configurable_digital_output_bits);
      data_pkg->getData(KEY_RUNTIME_STATE, runtime_state);
      data_pkg->getData(KEY_ELBOW_POSITION, elbow_position);
      data_pkg->getData(KEY_ELBOW_VELOCITY, elbow_velocity);
      data_pkg->getData(KEY_ROBOT_STATUS_BITS, robot_status_bits);
      data_pkg->getData(KEY_SAFETY_STATUS_BITS, safety_status_bits);
      data_pkg->getData(KEY_ANALOG_IO_TYPES, analog_io_types);
      data_pkg->getData(KEY_STANDARD_ANALOG_INPUT0, standard_analog_input0);
      data_pkg->getData(KEY_STANDARD_ANALOG_INPUT1, standard_analog_input1);
      data_pkg->getData(KEY_STANDARD_ANALOG_OUTPUT0, standard_analog_output0);
      data_pkg->getData(KEY_STANDARD_ANALOG_OUTPUT1, standard_analog_output1);
      data_pkg->getData(KEY_IO_CURRENT, io_current);
      data_pkg->getData(KEY_INPUT_BIT_REG_0_31, input_bit_reg_0_31);
      data_pkg->getData(KEY_INPUT_BIT_REG_32_63, input_bit_reg_32_63);
      data_pkg->getData(KEY_OUTPUT_BIT_REG_0_31, output_bit_reg_0_31);
      data_pkg->getData(KEY_OUTPUT_BIT_REG_32_63, output_bit_reg_32_63);
 
      for (int i = 64; i <= 127; ++i) {
          data_pkg->getData("input_bit_register_" + std::to_string(i), input_bit_register[i - 64]);
          data_pkg->getData("output_bit_register_" + std::to_string(i), output_bit_register[i - 64]);
      }
      for (int i = 0; i <= 47; ++i) {
          data_pkg->getData("input_int_register_" + std::to_string(i), input_int_register[i]);
          data_pkg->getData("input_double_register_" + std::to_string(i), input_double_register[i]);
          data_pkg->getData("output_int_register_" + std::to_string(i), output_int_register[i]);
          data_pkg->getData("output_double_register_" + std::to_string(i), output_double_register[i]);
      }
 
      data_pkg->getData(KEY_ACTUAL_ROBOT_ENERGY_CONSUMED, actual_robot_energy_consumed);
      data_pkg->getData(KEY_ACTUAL_ROBOT_BRAKING_ENERGY, actual_robot_braking_energy_dissipated);
      data_pkg->getData(KEY_ENCODER0_RAW, encoder0_raw);
      data_pkg->getData(KEY_ENCODER1_RAW, encoder1_raw);
      data_pkg->getData(KEY_EUROMAP67_INPUT_BITS, euromap67_input_bits);
      data_pkg->getData(KEY_EUROMAP67_OUTPUT_BITS, euromap67_output_bits);
      data_pkg->getData(KEY_EUROMAP67_24V_VOLTAGE, euromap67_24V_voltage);
      data_pkg->getData(KEY_EUROMAP67_24V_CURRENT, euromap67_24V_current);
      //data_pkg->getData(KEY_TOOL_MODE, tool_mode);
      //data_pkg->getData(KEY_TOOL_ANALOG_INPUT_TYPES, tool_analog_input_types);
      //data_pkg->getData(KEY_TOOL_ANALOG_INPUT0, tool_analog_input0);
      //data_pkg->getData(KEY_TOOL_ANALOG_INPUT1, tool_analog_input1);
      //data_pkg->getData(KEY_TOOL_OUTPUT_VOLTAGE, tool_output_voltage);
      //data_pkg->getData(KEY_TOOL_OUTPUT_CURRENT, tool_output_current);
      //data_pkg->getData(KEY_TOOL_TEMPERATURE, tool_temperature);
      //data_pkg->getData(KEY_TOOL_OUTPUT_MODE, tool_output_mode);
      //data_pkg->getData(KEY_TOOL_DIGITAL_OUTPUT0_MODE, tool_digital_output0_mode);
      //data_pkg->getData(KEY_TOOL_DIGITAL_OUTPUT1_MODE, tool_digital_output1_mode);
      data_pkg->getData(KEY_TCP_FORCE_SCALAR, tcp_force_scalar);
      data_pkg->getData(KEY_JOINT_POS_DEV_RATIO, joint_position_deviation_ratio);
      data_pkg->getData(KEY_COLLISION_DETECT_RATIO, collision_detection_ratio);
      data_pkg->getData(KEY_FT_RAW_WRENCH, ft_raw_wrench);
      data_pkg->getData(KEY_WRENCH_CALC_FROM_CURRENTS, wrench_calc_from_currents);
      data_pkg->getData(KEY_PAYLOAD, payload);
      data_pkg->getData(KEY_PAYLOAD_COG, payload_cog);
      data_pkg->getData(KEY_PAYLOAD_INERTIA, payload_inertia);
      data_pkg->getData(KEY_SCRIPT_CONTROL_LINE, script_control_line);
      data_pkg->getData(KEY_TIME_SCALE_SOURCE, time_scale_source);
      data_pkg->getData(KEY_TARGET_GRAVITY, target_gravity);
      data_pkg->getData(KEY_TARGET_BASE_ACCEL, target_base_acceleration);
 
      if (loop_counter % 500 == 0)
      {
        std::cout << "\r[RTDE] Todas las variables capturadas. TS: " << timestamp << "s   " << std::flush;
      }
    }
    else
    {
      std::cout << "Could not get fresh data package from robot" << std::endl;
      return 1;
    }
 
    if (speed_slider_increment > 0)
    {
      if (speed_slider_fraction + speed_slider_increment > 1.0)
        speed_slider_increment *= -1;
    }
    else
    {
      if (speed_slider_fraction + speed_slider_increment < 0.0)
        speed_slider_increment *= -1;
    }

    speed_slider_fraction += speed_slider_increment;

    if (!my_client.getWriter().sendSpeedSlider(speed_slider_fraction))
    {
      std::cerr << "\033[1;31m[ERROR] Failed to send speed slider.\033[0m\n";
      return 1;
    }

    digital_values = (digital_values == 0) ? digital_mask : 0;

    if (!my_client.getWriter().sendStandardDigitalOutput(digital_mask, digital_values))
    {
      std::cerr << "[ERROR] Failed to send standard digital output\n";
    }

    if (!my_client.getWriter().sendConfigurableDigitalOutput(digital_mask, digital_values))
    {
      std::cerr << "[ERROR] Failed to send configurable digital output\n";
    }

    if (analog_val + analog_increment > 1.0 || analog_val + analog_increment < 0.0)
    {
      analog_increment *= -1;
    }
    analog_val += analog_increment;

    if (!my_client.getWriter().sendStandardAnalogOutput(1, analog_val))
    {
      std::cerr << "[ERROR] Failed to send analog output\n";
    }

    int_register += 10;
    if (int_register > 100000) int_register = 0;

    double_register += 0.1;
    if (double_register > 100.0) double_register = 0.0;

    if (!my_client.getWriter().sendInputBitRegister(64, digital_values != 0))
    {
      std::cerr << "[ERROR] Failed to send input bit register\n";
    }

    if (!my_client.getWriter().sendInputIntRegister(24, int_register))
    {
      std::cerr << "[ERROR] Failed to send input int register\n";
    }

    if (!my_client.getWriter().sendInputDoubleRegister(24, double_register))
    {
      std::cerr << "[ERROR] Failed to send input double register\n";
    }

    if (force_val + force_increment > 10.0 || force_val + force_increment < -10.0)
    {
      force_increment *= -1;
    }
    force_val += force_increment;

    urcl::vector6d_t ext_wrench = {
      0.0,
      0.0,
      force_val,   
      0.0,
      0.0,
      0.0
    };

    if (!my_client.getWriter().sendExternalForceTorque(ext_wrench))
    {
      std::cerr << "[ERROR] Failed to send external wrench\n";
    }
 
    cycle_end = std::chrono::high_resolution_clock::now();
    g_durations_queue.enqueue(std::chrono::duration_cast<std::chrono::microseconds>(cycle_end - cycle_start).count());
    loop_counter++;
  }
 
  g_running = false;
  g_writer_thread.join();
 
  return 0;
}