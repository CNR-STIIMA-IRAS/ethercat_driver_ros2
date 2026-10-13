// Copyright 2022 ICUBE Laboratory, University of Strasbourg
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

#ifndef ETHERCAT_DRIVER__ETHERCAT_DRIVER_HPP_
#define ETHERCAT_DRIVER__ETHERCAT_DRIVER_HPP_

#include <chrono>
#include <unordered_map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <pluginlib/class_loader.hpp>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "ethercat_driver/visibility_control.h"
#include "ethercat_interface/ec_slave.hpp"
#include "ethercat_interface/ec_master.hpp"
#include "ethercat_msgs/srv/get_slave_states.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ethercat_driver
{

class EthercatDriver : public hardware_interface::SystemInterface
{
public:
  ETHERCAT_DRIVER_PUBLIC EthercatDriver();

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(EthercatDriver)

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  ETHERCAT_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ETHERCAT_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  ETHERCAT_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

public:
  ETHERCAT_DRIVER_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  ETHERCAT_DRIVER_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  std::vector<std::unordered_map<std::string, std::string>> getEcModuleParam(
    std::string & urdf, std::string component_name, std::string component_type);

  void get_slave_states_callback(
    const std::shared_ptr<ethercat_msgs::srv::GetSlaveStates::Request> request,
    std::shared_ptr<ethercat_msgs::srv::GetSlaveStates::Response> response);

  /** Bus supervision, called from read() after the process data has been received.
   *  While a bus fault is active, every exported "status_word" state interface reports
   *  kBusFaultStatusWord, which CiA402 decoders see as FAULT. */
  void superviseBus();
  void loadBusSupervisionParameters();
  void resetBusSupervision();

  struct CounterWatchdog
  {
    std::string name;
    const double * value = nullptr;
    double last_value = 0.0;
    bool initialized = false;
    std::chrono::steady_clock::time_point last_change;
  };

  std::vector<std::shared_ptr<ethercat_interface::EcSlave>> ec_modules_;
  std::vector<std::unordered_map<std::string, std::string>> ec_module_parameters_;

  std::vector<std::vector<double>> hw_joint_commands_;
  std::vector<std::vector<double>> hw_sensor_commands_;
  std::vector<std::vector<double>> hw_gpio_commands_;
  std::vector<std::vector<double>> hw_joint_states_;
  std::vector<std::vector<double>> hw_sensor_states_;
  std::vector<std::vector<double>> hw_gpio_states_;

  pluginlib::ClassLoader<ethercat_interface::EcSlave> ec_loader_{
    "ethercat_interface", "ethercat_interface::EcSlave"};

  int control_frequency_;

  int master_id_;
  ethercat_interface::EcMaster master_;
  rclcpp::Service<ethercat_msgs::srv::GetSlaveStates>::SharedPtr get_slave_states_srv_;
  std::mutex ec_mutex_;
  bool activated_;

  // Bus supervision parameters (hardware <param>, see loadBusSupervisionParameters()).
  int bus_fault_wc_cycles_ = 3;
  double bus_fault_hold_time_ = 1.0;
  double cycle_slow_factor_ = 3.0;
  double cycle_fault_max_gap_ = 0.05;
  double cycle_fault_ratio_ = 0.2;
  double cycle_fault_window_ = 1.0;
  double counter_watchdog_timeout_ = 0.05;

  // Bus supervision state, touched only by the control loop thread.
  std::vector<double *> status_word_states_;
  std::vector<CounterWatchdog> counter_watchdogs_;
  std::vector<bool> slave_seen_operational_;
  bool bus_armed_ = false;
  int wc_incomplete_cycles_ = 0;
  bool bus_fault_active_ = false;
  std::chrono::steady_clock::time_point bus_fault_last_seen_;
  bool have_last_read_ = false;
  std::chrono::steady_clock::time_point last_read_;
  std::chrono::steady_clock::time_point cycle_window_start_;
  uint32_t cycle_window_count_ = 0;
  uint32_t cycle_window_slow_ = 0;
};
}  // namespace ethercat_driver

#endif  // ETHERCAT_DRIVER__ETHERCAT_DRIVER_HPP_
