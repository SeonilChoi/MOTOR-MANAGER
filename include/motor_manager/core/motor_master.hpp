#ifndef CORE_MOTOR_MASTER_HPP_
#define CORE_MOTOR_MASTER_HPP_

#include <cstdint>

#include "motor_manager/types.hpp"

namespace micros {

class MotorMaster {
public:
    MotorMaster(const master_config_t& config)
    : id_(config.id),
      number_of_slaves_(config.number_of_slaves) {}
    virtual ~MotorMaster() = default;

    virtual void initialize() = 0;
    
    virtual void activate() = 0;
    
    virtual void deactivate() = 0;

    virtual void check() = 0;
    
    virtual void transmit() = 0;
    
    virtual void receive() = 0;
    
    uint8_t id() const { return id_; }
    
    uint8_t number_of_slaves() const { return number_of_slaves_; }

protected:
    const uint8_t id_;
    
    const uint8_t number_of_slaves_;
};

} //namespace micros
#endif // CORE_MOTOR_MASTER_HPP_