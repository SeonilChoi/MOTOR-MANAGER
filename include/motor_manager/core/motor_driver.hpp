#ifndef CORE_MOTOR_DRIVER_HPP_
#define CORE_MOTOR_DRIVER_HPP_

#include <string>
#include <cstddef>
#include <cstdint>

#include "motor_manager/types.hpp"

namespace micros {

class MotorDriver {
public:
    MotorDriver(const driver_config_t& config)
    : config_(config) {}
    virtual ~MotorDriver() = default;

    virtual void load_parameters(const std::string& param_file) = 0; 
    
    virtual bool is_enabled(const uint8_t* data, uint8_t* out) = 0;
    
    virtual bool is_disabled(const uint8_t* data, uint8_t* out) = 0;
    
    virtual bool is_received(const uint8_t* data, uint8_t* out) = 0;

    virtual double position(const int32_t value) = 0;
    
    virtual double velocity(const int32_t value) = 0;
    
    virtual double torque(const int16_t value) = 0;

    virtual int32_t position(const double value) = 0;
    
    virtual int32_t velocity(const double value) = 0;
    
    virtual int16_t torque(const double torque) = 0;

    const entry_table_t* items() const { return items_; }
    
    const entry_table_t* entries() const { return entries_; }
    
    std::size_t number_of_items() const { return number_of_items_; }
    
    std::size_t number_of_entries() const { return number_of_entries_; }
    
    std::size_t number_of_rx_pdos() const { return number_of_rx_pdos_; }
    
    std::size_t number_of_tx_pdos() const { return number_of_tx_pdos_; }

protected:
    entry_table_t items_[MAX_ITEM_SIZE];
    
    entry_table_t entries_[MAX_INTERFACE_SIZE];

    uint8_t number_of_items_{0};
    
    uint8_t number_of_entries_{0};

    uint8_t number_of_rx_pdos_{0};
    
    uint8_t number_of_tx_pdos_{0};

    const driver_config_t config_;
};

} //namespace micros
#endif // CORE_MOTOR_DRIVER_HPP_
