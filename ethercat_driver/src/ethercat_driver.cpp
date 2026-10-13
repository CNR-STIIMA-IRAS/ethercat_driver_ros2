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

#include "ethercat_driver/ethercat_driver.hpp"

#include <tinyxml2.h>
#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <regex>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ethercat_driver
{

namespace
{
// Reported on every "status_word" state interface while a bus fault is active.
// Decoded as FAULT by CiA402 consumers but never produced by a communicating drive (bit 9,
// "remote", is set whenever a drive reports a state), so supervisors can tell a bus fault
// from a drive fault.
constexpr double kBusFaultStatusWord = 0x0008;

double param_or(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & name, double default_value)
{
  const auto it = params.find(name);
  return it == params.end() ? default_value : std::stod(it->second);
}
}  // namespace

EthercatDriver::EthercatDriver()
: control_frequency_(-1)
  , master_id_(-1)
  , master_(-1, true)
  , activated_(false)
{}


CallbackReturn EthercatDriver::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Get master id
  if (info_.hardware_parameters.find("master_id") == info_.hardware_parameters.end()) {
    master_id_ = 0;
  } else {
    master_id_ = std::stod(info_.hardware_parameters["master_id"]);
  }
  bool ok = master_.setMaster(master_id_);
  if (!ok) {
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"),
      "Failed to connect to master %d", master_id_);
    return CallbackReturn::ERROR;
  }

  const std::lock_guard<std::mutex> lock(ec_mutex_);
  activated_ = false;

  hw_joint_states_.resize(info_.joints.size());
  for (uint j = 0; j < info_.joints.size(); j++) {
    hw_joint_states_[j].resize(
      info_.joints[j].state_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  hw_sensor_states_.resize(info_.sensors.size());
  for (uint s = 0; s < info_.sensors.size(); s++) {
    hw_sensor_states_[s].resize(
      info_.sensors[s].state_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  hw_gpio_states_.resize(info_.gpios.size());
  for (uint g = 0; g < info_.gpios.size(); g++) {
    hw_gpio_states_[g].resize(
      info_.gpios[g].state_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  hw_joint_commands_.resize(info_.joints.size());
  for (uint j = 0; j < info_.joints.size(); j++) {
    hw_joint_commands_[j].resize(
      info_.joints[j].command_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  hw_sensor_commands_.resize(info_.sensors.size());
  for (uint s = 0; s < info_.sensors.size(); s++) {
    hw_sensor_commands_[s].resize(
      info_.sensors[s].command_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  hw_gpio_commands_.resize(info_.gpios.size());
  for (uint g = 0; g < info_.gpios.size(); g++) {
    hw_gpio_commands_[g].resize(
      info_.gpios[g].command_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }

  for (uint j = 0; j < info_.joints.size(); j++) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "joints");
    // check all joints for EC modules and load into ec_modules_
    auto module_params = getEcModuleParam(info_.original_xml, info_.joints[j].name, "joint");
    ec_module_parameters_.insert(
      ec_module_parameters_.end(), module_params.begin(), module_params.end());
    for (auto i = 0ul; i < module_params.size(); i++) {
      for (auto k = 0ul; k < info_.joints[j].state_interfaces.size(); k++) {
        module_params[i]["state_interface/" +
          info_.joints[j].state_interfaces[k].name] = std::to_string(k);
      }
      for (auto k = 0ul; k < info_.joints[j].command_interfaces.size(); k++) {
        module_params[i]["command_interface/" +
          info_.joints[j].command_interfaces[k].name] = std::to_string(k);
      }
      try {
        auto module = ec_loader_.createSharedInstance(module_params[i].at("plugin"));
        if (!module->setupSlave(
            module_params[i], &hw_joint_states_[j], &hw_joint_commands_[j]))
        {
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"),
            "Setup of Joint module %li FAILED.", i + 1);
          return CallbackReturn::ERROR;
        }
        ec_modules_.push_back(module);
      } catch (pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(
          rclcpp::get_logger("EthercatDriver"),
          "The plugin of %s failed to load for some reason. Error: %s\n",
          info_.joints[j].name.c_str(), ex.what());
      }
    }
  }
  for (uint g = 0; g < info_.gpios.size(); g++) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "gpios");
    // check all gpios for EC modules and load into ec_modules_
    auto module_params = getEcModuleParam(info_.original_xml, info_.gpios[g].name, "gpio");
    ec_module_parameters_.insert(
      ec_module_parameters_.end(), module_params.begin(), module_params.end());
    for (auto i = 0ul; i < module_params.size(); i++) {
      for (auto k = 0ul; k < info_.gpios[g].state_interfaces.size(); k++) {
        module_params[i]["state_interface/" +
          info_.gpios[g].state_interfaces[k].name] = std::to_string(k);
      }
      for (auto k = 0ul; k < info_.gpios[g].command_interfaces.size(); k++) {
        module_params[i]["command_interface/" +
          info_.gpios[g].command_interfaces[k].name] = std::to_string(k);
      }
      try {
        auto module = ec_loader_.createSharedInstance(module_params[i].at("plugin"));
        if (!module->setupSlave(
            module_params[i], &hw_gpio_states_[g], &hw_gpio_commands_[g]))
        {
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"),
            "Setup of GPIO module %li FAILED.", i + 1);
          return CallbackReturn::ERROR;
        }
        ec_modules_.push_back(module);
      } catch (pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(
          rclcpp::get_logger("EthercatDriver"),
          "The plugin of %s failed to load for some reason. Error: %s\n",
          info_.gpios[g].name.c_str(), ex.what());
      }
    }
  }
  for (uint s = 0; s < info_.sensors.size(); s++) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "sensors");
    // check all sensors for EC modules and load into ec_modules_
    auto module_params = getEcModuleParam(info_.original_xml, info_.sensors[s].name, "sensor");
    ec_module_parameters_.insert(
      ec_module_parameters_.end(), module_params.begin(), module_params.end());
    for (auto i = 0ul; i < module_params.size(); i++) {
      for (auto k = 0ul; k < info_.sensors[s].state_interfaces.size(); k++) {
        module_params[i]["state_interface/" +
          info_.sensors[s].state_interfaces[k].name] = std::to_string(k);
      }
      for (auto k = 0ul; k < info_.sensors[s].command_interfaces.size(); k++) {
        module_params[i]["command_interface/" +
          info_.sensors[s].command_interfaces[k].name] = std::to_string(k);
      }
      try {
        auto module = ec_loader_.createSharedInstance(module_params[i].at("plugin"));
        if (!module->setupSlave(
            module_params[i], &hw_sensor_states_[s], &hw_sensor_commands_[s]))
        {
          RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"),
            "Setup of Sensor module %li FAILED.", i + 1);
          return CallbackReturn::ERROR;
        }
        ec_modules_.push_back(module);
      } catch (pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(
          rclcpp::get_logger("EthercatDriver"),
          "The plugin of %s failed to load for some reason. Error: %s\n",
          info_.sensors[s].name.c_str(), ex.what());
      }
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Got %li modules", ec_modules_.size());

  try {
    loadBusSupervisionParameters();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"),
      "Invalid bus supervision configuration: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

void EthercatDriver::loadBusSupervisionParameters()
{
  const auto & hw = info_.hardware_parameters;
  bus_fault_wc_cycles_ = static_cast<int>(param_or(hw, "bus_fault_wc_cycles", 3));
  bus_fault_hold_time_ = param_or(hw, "bus_fault_hold_time", 1.0);
  cycle_slow_factor_ = param_or(hw, "cycle_slow_factor", 3.0);
  cycle_fault_max_gap_ = param_or(hw, "cycle_fault_max_gap", 0.05);
  cycle_fault_ratio_ = param_or(hw, "cycle_fault_ratio", 0.2);
  cycle_fault_window_ = param_or(hw, "cycle_fault_window", 1.0);
  counter_watchdog_timeout_ = param_or(hw, "counter_watchdog_timeout", 0.05);

  // State vectors are never resized after on_init(), so the pointers stay valid.
  status_word_states_.clear();
  counter_watchdogs_.clear();
  auto collect = [this](
    const std::vector<hardware_interface::ComponentInfo> & components,
    std::vector<std::vector<double>> & states)
    {
      for (auto c = 0ul; c < components.size(); c++) {
        const auto & component = components[c];
        const auto wd_it = component.parameters.find("counter_watchdog");
        bool wd_found = wd_it == component.parameters.end();
        for (auto k = 0ul; k < component.state_interfaces.size(); k++) {
          const auto & name = component.state_interfaces[k].name;
          if (name == "status_word") {
            status_word_states_.push_back(&states[c][k]);
          }
          if (!wd_found && name == wd_it->second) {
            CounterWatchdog wd;
            wd.name = component.name + "/" + name;
            wd.value = &states[c][k];
            counter_watchdogs_.push_back(wd);
            wd_found = true;
          }
        }
        if (!wd_found) {
          throw std::runtime_error(
                  "counter_watchdog '" + wd_it->second + "' is not a state interface of " +
                  component.name);
        }
      }
    };
  collect(info_.joints, hw_joint_states_);
  collect(info_.sensors, hw_sensor_states_);
  collect(info_.gpios, hw_gpio_states_);

  RCLCPP_INFO(
    rclcpp::get_logger("EthercatDriver"),
    "Bus supervision: wc_cycles=%d hold=%.3fs slow_factor=%.1f max_gap=%.3fs "
    "slow_ratio=%.2f/%.1fs counter_timeout=%.3fs, %zu status words, %zu counter watchdogs",
    bus_fault_wc_cycles_, bus_fault_hold_time_, cycle_slow_factor_, cycle_fault_max_gap_,
    cycle_fault_ratio_, cycle_fault_window_, counter_watchdog_timeout_,
    status_word_states_.size(), counter_watchdogs_.size());
}

void EthercatDriver::resetBusSupervision()
{
  slave_seen_operational_.assign(ec_modules_.size(), false);
  for (auto & wd : counter_watchdogs_) {
    wd.initialized = false;
  }
  bus_armed_ = false;
  wc_incomplete_cycles_ = 0;
  bus_fault_active_ = false;
  have_last_read_ = false;
  cycle_window_count_ = 0;
  cycle_window_slow_ = 0;
}

void EthercatDriver::superviseBus()
{
  using std::chrono::duration;
  const auto logger = rclcpp::get_logger("EthercatDriver");
  const auto now = std::chrono::steady_clock::now();

  // The reason string is built only when a fault starts, to keep the loop allocation-free.
  enum class Cause { NONE, CYCLE_GAP, SLOW_CYCLES, WORKING_COUNTER, SLAVE, COUNTER };
  Cause cause = Cause::NONE;
  double cause_value = 0.0;
  std::size_t cause_index = 0;

  // Working counter: per-cycle, domain-wide. Every check is armed once the bus was complete,
  // so the slaves' gradual transition to OP at startup is not reported as a fault.
  const ec_wc_state_t wc = master_.getDomainWcState();
  if (wc == EC_WC_COMPLETE) {
    wc_incomplete_cycles_ = 0;
    bus_armed_ = true;
  } else if (bus_armed_ && ++wc_incomplete_cycles_ >= bus_fault_wc_cycles_) {
    cause = Cause::WORKING_COUNTER;
    cause_value = wc;
  }

  // Control loop periodicity: everything the drives see is paced by read()/write().
  if (have_last_read_ && bus_armed_) {
    const double dt = duration<double>(now - last_read_).count();
    const double period = 1.0 / control_frequency_;
    if (cycle_fault_max_gap_ > 0.0 && dt > cycle_fault_max_gap_ && cause == Cause::NONE) {
      cause = Cause::CYCLE_GAP;
      cause_value = dt;
    }
    if (dt > cycle_slow_factor_ * period) {
      cycle_window_slow_++;
      RCLCPP_WARN_THROTTLE(
        logger, *get_clock(), 1000,
        "Slow control cycle on master %d: %.1f ms (nominal %.1f ms)",
        master_id_, dt * 1e3, period * 1e3);
    }
    cycle_window_count_++;
    if (duration<double>(now - cycle_window_start_).count() >= cycle_fault_window_) {
      const double ratio = static_cast<double>(cycle_window_slow_) / cycle_window_count_;
      if (cycle_fault_ratio_ > 0.0 && ratio > cycle_fault_ratio_ && cause == Cause::NONE) {
        cause = Cause::SLOW_CYCLES;
        cause_value = ratio;
      }
      cycle_window_start_ = now;
      cycle_window_count_ = 0;
      cycle_window_slow_ = 0;
    }
  } else {
    cycle_window_start_ = now;
  }
  have_last_read_ = true;
  last_read_ = now;

  // Per-slave AL state, refreshed every few cycles; armed per slave once seen in OP.
  for (auto i = 0ul; i < ec_modules_.size(); i++) {
    if (ec_modules_[i]->slave_online_and_operational()) {
      slave_seen_operational_[i] = true;
    } else if (slave_seen_operational_[i] && cause == Cause::NONE) {
      cause = Cause::SLAVE;
      cause_index = i;
    }
  }

  // Counters that a healthy device advances every cycle (e.g. a sensor sample counter).
  if (bus_armed_) {
    for (auto i = 0ul; i < counter_watchdogs_.size(); i++) {
      auto & wd = counter_watchdogs_[i];
      const double value = *wd.value;
      if (!wd.initialized || value != wd.last_value) {
        wd.initialized = true;
        wd.last_value = value;
        wd.last_change = now;
      } else if (duration<double>(now - wd.last_change).count() > counter_watchdog_timeout_ &&
        cause == Cause::NONE)
      {
        cause = Cause::COUNTER;
        cause_index = i;
      }
    }
  }

  if (cause != Cause::NONE) {
    if (!bus_fault_active_) {
      std::string reason;
      switch (cause) {
        case Cause::CYCLE_GAP:
          reason = "control cycle gap of " + std::to_string(cause_value * 1e3) + " ms";
          break;
        case Cause::SLOW_CYCLES:
          reason = std::to_string(cause_value * 100.0) + "% slow control cycles";
          break;
        case Cause::WORKING_COUNTER:
          reason = "domain working counter incomplete for " +
            std::to_string(wc_incomplete_cycles_) + " cycles (wc_state=" +
            std::to_string(static_cast<int>(cause_value)) + ")";
          break;
        case Cause::SLAVE: {
            const auto state = ec_modules_[cause_index]->get_slave_state();
            reason = "slave " + state.name + " at position " + std::to_string(state.position) +
              (state.online ? " not operational (AL " +
              ethercat_interface::EcSlave::al_state_to_string(state.al_state) + ")" :
              " offline");
            break;
          }
        case Cause::COUNTER:
          reason = counter_watchdogs_[cause_index].name + " frozen for more than " +
            std::to_string(counter_watchdog_timeout_ * 1e3) + " ms";
          break;
        case Cause::NONE:
          break;
      }
      RCLCPP_ERROR(
        logger, "Bus fault on master %d: %s. Reporting FAULT on %zu drive status words.",
        master_id_, reason.c_str(), status_word_states_.size());
      bus_fault_active_ = true;
    }
    bus_fault_last_seen_ = now;
  } else if (bus_fault_active_ &&
    duration<double>(now - bus_fault_last_seen_).count() > bus_fault_hold_time_)
  {
    RCLCPP_WARN(logger, "Bus fault on master %d cleared.", master_id_);
    bus_fault_active_ = false;
  }

  // Held for at least bus_fault_hold_time so that sampled supervisors cannot miss it.
  if (bus_fault_active_) {
    for (double * status_word : status_word_states_) {
      *status_word = kBusFaultStatusWord;
    }
  }
}

CallbackReturn EthercatDriver::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  using namespace std::placeholders;
  get_slave_states_srv_ = get_node()->create_service<ethercat_msgs::srv::GetSlaveStates>(
    "~/get_slave_states_master" + std::to_string(master_id_),
    std::bind(&EthercatDriver::get_slave_states_callback, this, _1, _2));
  return CallbackReturn::SUCCESS;
}

void EthercatDriver::get_slave_states_callback(
  const std::shared_ptr<ethercat_msgs::srv::GetSlaveStates::Request> /*request*/,
  std::shared_ptr<ethercat_msgs::srv::GetSlaveStates::Response> response)
{
  response->slave_names.clear();
  response->slave_states.clear();
  response->slave_names.reserve(ec_modules_.size());
  response->slave_states.reserve(ec_modules_.size());

  for (auto i = 0ul; i < ec_modules_.size(); ++i) {
    const auto state = ec_modules_[i]->get_slave_state();
    std::string slave_name = state.name;
    if (slave_name.empty() && i < ec_module_parameters_.size()) {
      const auto name_it = ec_module_parameters_[i].find("name");
      if (name_it != ec_module_parameters_[i].end()) {
        slave_name = name_it->second;
      }
    }
    if (slave_name.empty()) {
      slave_name = "slave";
    }

    response->slave_names.push_back(
      slave_name + "-" + std::to_string(state.alias) + ":" + std::to_string(state.position));
    response->slave_states.push_back(
      state.online ? ethercat_interface::EcSlave::al_state_to_string(state.al_state) : "OFFLINE");
  }
}

std::vector<hardware_interface::StateInterface>
EthercatDriver::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  // export joint state interface
  for (uint j = 0; j < info_.joints.size(); j++) {
    for (uint i = 0; i < info_.joints[j].state_interfaces.size(); i++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.joints[j].name,
          info_.joints[j].state_interfaces[i].name,
          &hw_joint_states_[j][i]));
    }
  }
  // export sensor state interface
  for (uint s = 0; s < info_.sensors.size(); s++) {
    for (uint i = 0; i < info_.sensors[s].state_interfaces.size(); i++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.sensors[s].name,
          info_.sensors[s].state_interfaces[i].name,
          &hw_sensor_states_[s][i]));
    }
  }
  // export gpio state interface
  for (uint g = 0; g < info_.gpios.size(); g++) {
    for (uint i = 0; i < info_.gpios[g].state_interfaces.size(); i++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.gpios[g].name,
          info_.gpios[g].state_interfaces[i].name,
          &hw_gpio_states_[g][i]));
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
EthercatDriver::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // export joint command interface
  std::vector<double> test;
  for (uint j = 0; j < info_.joints.size(); j++) {
    for (uint i = 0; i < info_.joints[j].command_interfaces.size(); i++) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[j].name,
          info_.joints[j].command_interfaces[i].name,
          &hw_joint_commands_[j][i]));
    }
  }
  // export sensor command interface
  for (uint s = 0; s < info_.sensors.size(); s++) {
    for (uint i = 0; i < info_.sensors[s].command_interfaces.size(); i++) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.sensors[s].name,
          info_.sensors[s].command_interfaces[i].name,
          &hw_sensor_commands_[s][i]));
    }
  }
  // export gpio command interface
  for (uint g = 0; g < info_.gpios.size(); g++) {
    for (uint i = 0; i < info_.gpios[g].command_interfaces.size(); i++) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.gpios[g].name,
          info_.gpios[g].command_interfaces[i].name,
          &hw_gpio_commands_[g][i]));
    }
  }
  return command_interfaces;
}

CallbackReturn EthercatDriver::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const std::lock_guard<std::mutex> lock(ec_mutex_);
  if (activated_) {
    RCLCPP_FATAL(rclcpp::get_logger("EthercatDriver"), "Double on_activate()");
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Starting ...please wait...");
  if (info_.hardware_parameters.find("control_frequency") == info_.hardware_parameters.end()) {
    control_frequency_ = 100;
  } else {
    control_frequency_ = std::stod(info_.hardware_parameters["control_frequency"]);
  }

  if (control_frequency_ < 0) {
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"), "Invalid control frequency!");
    return CallbackReturn::ERROR;
  }

  // start EC and wait until state operative

  master_.setCtrlFrequency(control_frequency_);

  for (auto i = 0ul; i < ec_modules_.size(); i++) {
    master_.addSlave(
      std::stod(ec_module_parameters_[i]["alias"]),
      std::stod(ec_module_parameters_[i]["position"]),
      ec_modules_[i].get());
  }

  // configure SDO
  for (auto i = 0ul; i < ec_modules_.size(); i++) {
    for (auto & sdo : ec_modules_[i]->sdo_config) {
      uint32_t abort_code;
      int ret = master_.configSlaveSdo(
        std::stod(ec_module_parameters_[i]["position"]),
        sdo,
        &abort_code
      );
      if (ret) {
        RCLCPP_INFO(
          rclcpp::get_logger("EthercatDriver"),
          "Failed to download config SDO for module at position %s with Error: %d",
          ec_module_parameters_[i]["position"].c_str(),
          abort_code
        );
      }
    }
  }

  if (!master_.activate()) {
    RCLCPP_ERROR(rclcpp::get_logger("EthercatDriver"), "Activate EcMaster failed");
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Activated EcMaster!");

  // start after one second
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  t.tv_sec++;

  bool running = true;
  while (running) {
    // wait until next shot
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
    // update EtherCAT bus

    master_.update();
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "updated!");

    // check if operational
    bool isAllInit = true;
    for (auto & module : ec_modules_) {
      isAllInit = isAllInit && module->initialized();
    }
    if (isAllInit) {
      running = false;
    }
    // calculate next shot. carry over nanoseconds into microseconds.
    t.tv_nsec += master_.getInterval();
    while (t.tv_nsec >= 1000000000) {
      t.tv_nsec -= 1000000000;
      t.tv_sec++;
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("EthercatDriver"), "System Successfully started!");

  resetBusSupervision();
  activated_ = true;

  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatDriver::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const std::lock_guard<std::mutex> lock(ec_mutex_);
  activated_ = false;

  RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Stopping ...please wait...");

  // stop EC and disconnect
  master_.stop();

  RCLCPP_INFO(
    rclcpp::get_logger("EthercatDriver"), "System successfully stopped!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type EthercatDriver::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // try to lock so we can avoid blocking the read/write loop on the lock.
  const std::unique_lock<std::mutex> lock(ec_mutex_, std::try_to_lock);
  if (lock.owns_lock() && activated_) {
    master_.readData();
    superviseBus();
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EthercatDriver::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  // try to lock so we can avoid blocking the read/write loop on the lock.
  const std::unique_lock<std::mutex> lock(ec_mutex_, std::try_to_lock);
  if (lock.owns_lock() && activated_) {
    master_.writeData();
  }
  return hardware_interface::return_type::OK;
}

std::vector<std::unordered_map<std::string, std::string>> EthercatDriver::getEcModuleParam(
  std::string & urdf,
  std::string component_name,
  std::string component_type)
{
  // Check if everything OK with URDF string
  if (urdf.empty()) {
    throw std::runtime_error("empty URDF passed to robot");
  }
  tinyxml2::XMLDocument doc;
  if (!doc.Parse(urdf.c_str()) && doc.Error()) {
    throw std::runtime_error("invalid URDF passed in to robot parser");
  }
  if (doc.Error()) {
    throw std::runtime_error("invalid URDF passed in to robot parser");
  }

  tinyxml2::XMLElement * robot_it = doc.RootElement();
  if (std::string("robot").compare(robot_it->Name())) {
    throw std::runtime_error("the robot tag is not root element in URDF");
  }

  const tinyxml2::XMLElement * ros2_control_it = robot_it->FirstChildElement("ros2_control");
  if (!ros2_control_it) {
    throw std::runtime_error("no ros2_control tag");
  }

  std::vector<std::unordered_map<std::string, std::string>> module_params;
  std::unordered_map<std::string, std::string> module_param;

  while (ros2_control_it) {
    const auto * ros2_control_child_it = ros2_control_it->FirstChildElement(component_type.c_str());
    while (ros2_control_child_it) {
      if (!component_name.compare(ros2_control_child_it->Attribute("name"))) {
        const auto * ec_module_it = ros2_control_child_it->FirstChildElement("ec_module");
        while (ec_module_it) {
          module_param.clear();
          module_param["name"] = ec_module_it->Attribute("name");
          const auto * plugin_it = ec_module_it->FirstChildElement("plugin");
          if (NULL != plugin_it) {
            module_param["plugin"] = plugin_it->GetText();
          }
          const auto * param_it = ec_module_it->FirstChildElement("param");
          while (param_it) {
            module_param[param_it->Attribute("name")] = param_it->GetText();
            param_it = param_it->NextSiblingElement("param");
          }
          module_params.push_back(module_param);
          ec_module_it = ec_module_it->NextSiblingElement("ec_module");
        }
      }
      ros2_control_child_it = ros2_control_child_it->NextSiblingElement(component_type.c_str());
    }
    ros2_control_it = ros2_control_it->NextSiblingElement("ros2_control");
  }

  return module_params;
}

}  // namespace ethercat_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ethercat_driver::EthercatDriver, hardware_interface::SystemInterface)
