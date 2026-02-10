#include <cmath>
#include <string>
#include <cstdio>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "motor_manager/driver/minas_driver.hpp"

void mmns::MinasDriver::load_parameters(const std::string& param_file)
{
    YAML::Node root = YAML::LoadFile(param_file);
    if (!root) {
        throw std::runtime_error("Failed to load parameter file.");
    }

    YAML::Node objects = root["objects"];
    if (!objects || !objects.IsSequence()) {
        throw std::runtime_error("Invalid objects configuration.");
    }

    uint8_t o_idx{0};
    for (const auto& o : objects) {
        entry_table_t e_cfg{};
        e_cfg.id = o["id"].as<uint8_t>();
        e_cfg.index = o["index"].as<uint16_t>();
        e_cfg.subindex = o["subindex"].as<uint8_t>();
        e_cfg.type = to_value_type(o["type"].as<std::string>());
        
        if (e_cfg.id == ID_MAX_TORQUE) {
            uint16_t value = static_cast<uint16_t>(2.0 / config_.unit_torque * 100.0);
            fill_data<uint16_t>(value, e_cfg.data);
        } else if (e_cfg.id == ID_MIN_POSITION_LIMIT) {
            int32_t value = static_cast<int32_t>(config_.lower / (2 * M_PI) * config_.pulse_per_revolution);
            fill_data<int32_t>(value, e_cfg.data);
        } else if (e_cfg.id == ID_MAX_POSITION_LIMIT) {
            int32_t value = static_cast<int32_t>(config_.upper / (2 * M_PI) * config_.pulse_per_revolution);
            fill_data<int32_t>(value, e_cfg.data);
        } else if (e_cfg.id == ID_MAX_MOTOR_SPEED) {
            fill_data<uint32_t>(static_cast<uint32_t>(config_.speed), e_cfg.data);
        } else if (e_cfg.id == ID_PROFILE_VELOCITY) {
            uint32_t value = static_cast<uint32_t>(config_.profile_velocity / (2 * M_PI) * config_.pulse_per_revolution);
            fill_data<uint32_t>(static_cast<uint32_t>(value), e_cfg.data);
        } else if (e_cfg.id == ID_PROFILE_ACCELERATION) {
            uint32_t value = static_cast<uint32_t>(config_.profile_acceleration / (2 * M_PI) * config_.pulse_per_revolution);
            fill_data<uint32_t>(static_cast<uint32_t>(value), e_cfg.data);
        } else if (e_cfg.id == ID_PROFILE_DECELERATION) {
            uint32_t value = static_cast<uint32_t>(config_.profile_deceleration / (2 * M_PI) * config_.pulse_per_revolution);
            fill_data<uint32_t>(static_cast<uint32_t>(value), e_cfg.data);
        } else if (e_cfg.id == ID_MAX_ACCELERATION) {
            uint32_t value = static_cast<uint32_t>(config_.acceleration / (2 * M_PI) * config_.pulse_per_revolution);
            fill_data<uint32_t>(static_cast<uint32_t>(value), e_cfg.data);
        } else if (e_cfg.id == ID_MAX_DECELERATION) {
            uint32_t value = static_cast<uint32_t>(config_.deceleration / (2 * M_PI) * config_.pulse_per_revolution);
            fill_data<uint32_t>(static_cast<uint32_t>(value), e_cfg.data);
        } else {
            switch (e_cfg.type) {
            case ValueType::U8: {
                fill_data<uint8_t>(o["value"].as<uint8_t>(), e_cfg.data);
                break;
            } case ValueType::U16: {
                fill_data<uint16_t>(o["value"].as<uint16_t>(), e_cfg.data);
                break;
            } case ValueType::U32: {
                fill_data<uint32_t>(o["value"].as<uint32_t>(), e_cfg.data);
                break;
            } case ValueType::S8: {
                fill_data<int8_t>(o["value"].as<int8_t>(), e_cfg.data);
                break;
            } case ValueType::S16: {
                fill_data<int16_t>(o["value"].as<int16_t>(), e_cfg.data);
                break;
            } case ValueType::S32: {
                fill_data<int32_t>(o["value"].as<int32_t>(), e_cfg.data);
                break;
            } default: {
                throw std::runtime_error("Unsupported data type of object.");
            }
            }
        }
        
        items_[o_idx] = e_cfg;
        o_idx++;
    }
    number_of_items_ = o_idx;

    YAML::Node entries = root["entries"];
    if (!entries || !entries.IsSequence()) {
        throw std::runtime_error("Invalid entries configuration.");
    }

    uint8_t e_idx{0}, c_idx{0}, s_idx{0};
    for (const auto& e : entries) {
        entry_table_t e_cfg{};
        e_cfg.id = e["id"].as<uint8_t>();
        e_cfg.index = e["index"].as<uint16_t>();

        if (e_cfg.id != ID_RXPDO && e_cfg.id != ID_TXPDO) {
            e_cfg.subindex = e["subindex"].as<uint8_t>();
            e_cfg.size = e["size"].as<uint8_t>();
            e_cfg.type = to_value_type(e["type"].as<std::string>());

            if (e_cfg.id <= ID_TARGET_TORQUE) {
                c_idx++;
            } else {
                s_idx++;
            }
        }
        entries_[e_idx] = e_cfg;
        e_idx++;
    }
    number_of_entries_ = e_idx;
    number_of_rx_pdos_ = c_idx; // motor receive command from motor manager
    number_of_tx_pdos_ = s_idx; // motor transmit state to motor manager

    printf("[MinasDriver::load_parameters][driver id: %u] Parameter load succeed.\n", config_.id);
}

bool mmns::MinasDriver::is_enabled(const uint8_t* data, uint8_t* out)
{
    uint16_t sw = to_value<uint16_t>(data);
    uint16_t cw = CW_ENABLE_OPERAITON;
    
    if (is_fault(sw)) {
        cw = CW_FAULT_RESET;
        driver_state_ = DriverState::SwitchOnDisabled;
    }

    switch (driver_state_) {
    case DriverState::SwitchOnDisabled: {
        cw = CW_SHUTDOWN;
        if (is_ready_to_switch_on(sw)) {
            driver_state_ = DriverState::ReadyToSwitchOn;
        }
        break;
    } case DriverState::ReadyToSwitchOn: {
        cw = CW_SWITCH_ON;
        if (is_switched_on(sw)) {
            driver_state_ = DriverState::SwitchedOn;
        }
        break;
    } case DriverState::SwitchedOn: {
        cw = CW_ENABLE_OPERAITON;
        if (is_operation_enabled(sw)) {
            driver_state_ = DriverState::OperationEnabled;
        }
        break;
    } case DriverState::OperationEnabled: {
        printf("[MinasDriver::is_enabled][driver id: %u] Operation enabled.\n", config_.id);
        return true;
    } default: {
        break;
    }
    }

    fill_data<uint16_t>(cw, out);
    return false;
}

bool mmns::MinasDriver::is_disabled(const uint8_t* data, uint8_t* out)
{
    uint16_t sw = to_value<uint16_t>(data);
    uint16_t cw = CW_ENABLE_OPERAITON;
    
    switch (driver_state_) {
    case DriverState::SwitchOnDisabled: {
        printf("[MinasDriver::is_disabled][driver id: %u] Operation disabled.\n", config_.id);
        return true;
    }
    case DriverState::ReadyToSwitchOn: {
        cw = CW_DISABLE_VOLTAGE;
        if (is_switch_on_disabled(sw)) {
            driver_state_ = DriverState::SwitchOnDisabled;
        }
        break;
    }
    case DriverState::SwitchedOn: {
        cw = CW_SHUTDOWN;
        if (is_ready_to_switch_on(sw)) {
            driver_state_ = DriverState::ReadyToSwitchOn;
        }
        break;
    }
    case DriverState::OperationEnabled: {
        cw = CW_DISABLE_OPERATION;
        if (is_switched_on(sw)) {
            driver_state_ = DriverState::SwitchedOn;
        }
        break;
    }
    default:
        break;
    }
    fill_data<uint16_t>(cw, out);
    return false;
}

bool mmns::MinasDriver::is_received(const uint8_t* data, uint8_t* out)
{
    uint16_t sw = to_value<uint16_t>(data);
    if (is_setpoint_acknowledge(sw)) {
        fill_data<uint16_t>(0x0F, out);
        return true;
    }
    return false;
}

double mmns::MinasDriver::position(const int32_t value)
{
    return static_cast<double>(value) / static_cast<double>(config_.pulse_per_revolution) * (2 * M_PI);
} 

double mmns::MinasDriver::velocity(const int32_t value)
{
    return static_cast<double>(value) / static_cast<double>(config_.pulse_per_revolution) * (2 * M_PI);
} 

double mmns::MinasDriver::torque(const int16_t value)
{
    return config_.rated_torque * 0.01 * static_cast<double>(value) * config_.unit_torque;
}

int32_t mmns::MinasDriver::position(const double value)
{
    return static_cast<int32_t>(value / (2 * M_PI) * config_.pulse_per_revolution); 
} 

int32_t mmns::MinasDriver::velocity(const double value)
{
    return static_cast<int32_t>(value / (2 * M_PI) * config_.pulse_per_revolution); 
} 

int16_t mmns::MinasDriver::torque(const double value)
{
    return static_cast<int16_t>(value / config_.rated_torque * 100 / config_.unit_torque);
}