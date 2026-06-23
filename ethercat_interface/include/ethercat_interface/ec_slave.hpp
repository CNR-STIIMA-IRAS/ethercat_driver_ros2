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

#ifndef ETHERCAT_INTERFACE__EC_SLAVE_HPP_
#define ETHERCAT_INTERFACE__EC_SLAVE_HPP_

#include <ecrt.h>
#include <atomic>
#include <cstdint>
#include <map>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <cmath>
#include <string>

#include "ethercat_interface/ec_sdo_manager.hpp"

namespace ethercat_interface
{

struct EcSlaveState
{
  std::string name;
  uint16_t alias = 0;
  uint16_t position = 0;
  uint32_t vendor_id = 0;
  uint32_t product_id = 0;
  uint8_t al_state = 0;
  bool online = false;
  bool operational = false;
};

class EcSlave
{
public:
  EcSlave(uint32_t vendor_id, uint32_t product_id)
  : vendor_id_(vendor_id),
    product_id_(product_id) {}
  virtual ~EcSlave() {}
  /** read or write data to the domain */
  virtual void processData(size_t /*index*/, uint8_t * /*domain_address*/) {}
  /** a pointer to syncs. return &syncs[0] */
  virtual const ec_sync_info_t * syncs() {return NULL;}
  virtual bool initialized() {return true;}
  virtual void set_state_is_operational(bool value)
  {
    is_operational_ = value;
    operational_.store(value);
  }
  /** Assign activate DC synchronization. return activate word*/
  virtual int assign_activate_dc_sync() {return 0x00;}
  /** number of elements in the syncs array. */
  virtual size_t syncSize() {return 0;}
  /** a pointer to all PDO entries */
  virtual const ec_pdo_entry_info_t * channels() {return NULL;}
  /** a map from domain index to pdo indices in that domain.
  *  map<domain index, vector<channels_ indices> > */
  typedef std::map<unsigned int, std::vector<unsigned int>> DomainMap;
  virtual void domains(DomainMap & /*domains*/) const {}

  void set_slave_metadata(const std::unordered_map<std::string, std::string> & parameters)
  {
    const auto name_it = parameters.find("name");
    if (name_it != parameters.end()) {
      set_slave_name(name_it->second);
    }

    const auto alias_it = parameters.find("alias");
    const auto position_it = parameters.find("position");
    if (alias_it != parameters.end() && position_it != parameters.end()) {
      set_slave_identity(
        static_cast<uint16_t>(std::stoul(alias_it->second)),
        static_cast<uint16_t>(std::stoul(position_it->second)));
    }
  }

  void set_slave_identity(uint16_t alias, uint16_t position)
  {
    alias_.store(alias);
    position_.store(position);
  }

  void set_slave_name(const std::string & name)
  {
    slave_name_ = name;
  }

  void set_slave_config_state(const ec_slave_config_state_t & state)
  {
    al_state_.store(state.al_state);
    online_.store(state.online);
    operational_.store(state.operational);
    is_operational_ = state.operational;
  }

  EcSlaveState get_slave_state() const
  {
    return EcSlaveState{
      slave_name_,
      alias_.load(),
      position_.load(),
      vendor_id_,
      product_id_,
      al_state_.load(),
      online_.load(),
      operational_.load()};
  }

  static std::string al_state_to_string(uint8_t al_state)
  {
    switch (al_state & 0x0F) {
      case EC_AL_STATE_INIT:
        return "INIT";
      case EC_AL_STATE_PREOP:
        return "PREOP";
      case EC_AL_STATE_SAFEOP:
        return "SAFEOP";
      case EC_AL_STATE_OP:
        return "OP";
      default:
        return "UNKNOWN";
    }
  }

  virtual bool setupSlave(
    std::unordered_map<std::string, std::string> slave_paramters,
    std::vector<double> * state_interface,
    std::vector<double> * command_interface)
  {
    state_interface_ptr_ = state_interface;
    command_interface_ptr_ = command_interface;
    paramters_ = slave_paramters;
    set_slave_metadata(paramters_);
    return true;
  }
  uint32_t vendor_id_;
  uint32_t product_id_;

  std::vector<SdoConfigEntry> sdo_config;

protected:
  std::vector<double> * state_interface_ptr_;
  std::vector<double> * command_interface_ptr_;
  std::unordered_map<std::string, std::string> paramters_;
  bool is_operational_ = false;
  std::string slave_name_;
  std::atomic<uint16_t> alias_{0};
  std::atomic<uint16_t> position_{0};
  std::atomic<uint8_t> al_state_{0};
  std::atomic<bool> online_{false};
  std::atomic<bool> operational_{false};
};
}  // namespace ethercat_interface
#endif  // ETHERCAT_INTERFACE__EC_SLAVE_HPP_
