#ifndef CORE_MOTOR_CONTROLLER_HPP_
#define CORE_MOTOR_CONTROLLER_HPP_

#include <cstddef>
#include <cstdint>

#include "motor_manager/core/motor_master.hpp"
#include "motor_manager/core/motor_driver.hpp"

namespace mmns {

class MotorController {
public:
    explicit MotorController(const slave_config_t& config)
    : master_id_(config.master_id),
      driver_id_(config.driver_id) {}
    virtual ~MotorController() = default;

    virtual void initialize(MotorMaster& master, MotorDriver& driver) = 0;
    
    virtual void configure() = 0;
    
    virtual bool servo_on() = 0;
    
    virtual bool servo_off() = 0;

    virtual void check(const motor_state_t& state) = 0;
    
    virtual void write(const motor_state_t& cmd) = 0;
    
    virtual void read(motor_state_t& state) = 0;

    uint8_t master_id() { return master_id_; }
    
    uint8_t driver_id() { return driver_id_; }

protected:
    virtual void write_data(const entry_table_t* pdos, uint8_t size) = 0;
    
    virtual void read_data(entry_table_t* pdos) = 0;

    const uint8_t master_id_;
    
    const uint8_t driver_id_;
};

} //namespace mmns
#endif