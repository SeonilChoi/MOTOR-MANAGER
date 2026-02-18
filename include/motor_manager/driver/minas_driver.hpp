#ifndef DRIVER_MINAS_DRIVER_HPP_
#define DRIVER_MINAS_DRIVER_HPP_

#include <cstdint>

#include "motor_manager/core/motor_driver.hpp"

inline constexpr uint16_t SW_FAULT                = 0x0008;
inline constexpr uint16_t SW_READY_TO_SWITCH_ON   = 0x0021;
inline constexpr uint16_t SW_SWITCHED_ON          = 0x0023;
inline constexpr uint16_t SW_OPERATION_ENABLED    = 0x0027;
inline constexpr uint16_t SW_SWITCH_ON_DISABLED   = 0x0040;
inline constexpr uint16_t SW_SETPOINT_ACKNOWLEDGE = 0x1000;

inline constexpr uint16_t CW_DISABLE_VOLTAGE   = 0x0000;
inline constexpr uint16_t CW_SHUTDOWN          = 0x0006;
inline constexpr uint16_t CW_SWITCH_ON         = 0x0007;
inline constexpr uint16_t CW_DISABLE_OPERATION = 0x0007;
inline constexpr uint16_t CW_ENABLE_OPERAITON  = 0x000F;
inline constexpr uint16_t CW_NEW_SETPOINT      = 0x003F;
inline constexpr uint16_t CW_FAULT_RESET       = 0x0080;

inline constexpr uint8_t ID_MAX_TORQUE           = 50;
inline constexpr uint8_t ID_MIN_POSITION_LIMIT   = 51;
inline constexpr uint8_t ID_MAX_POSITION_LIMIT   = 52;
inline constexpr uint8_t ID_MAX_MOTOR_SPEED      = 53;
inline constexpr uint8_t ID_PROFILE_VELOCITY     = 54;
inline constexpr uint8_t ID_PROFILE_ACCELERATION = 55;
inline constexpr uint8_t ID_PROFILE_DECELERATION = 56;
inline constexpr uint8_t ID_MAX_ACCELERATION     = 57;
inline constexpr uint8_t ID_MAX_DECELERATION     = 58;
inline constexpr uint8_t ID_RXPDO                = 98;
inline constexpr uint8_t ID_TXPDO                = 99;

namespace micros {

class MinasDriver : public MotorDriver {
public:
    MinasDriver(const driver_config_t& config)
    : MotorDriver(config) {}
    ~MinasDriver() = default;

    void load_parameters(const std::string& param_file) override;

    bool is_enabled(const uint8_t* data, DriverState& driver_state, uint8_t* out) override; 
    
    bool is_disabled(const uint8_t* data, DriverState& driver_state, uint8_t* out) override;
    
    bool is_received(const uint8_t* data, uint8_t* out) override;

    double position(const int32_t value) override;
    
    double velocity(const int32_t value) override;
    
    double torque(const int16_t value) override;

    int32_t position(const double value) override;
    
    int32_t velocity(const double value) override;
    
    int16_t torque(const double value) override;

private:
    bool is_ready_to_switch_on(uint16_t sw) { return (sw & 0x006F) == SW_READY_TO_SWITCH_ON; }
    
    bool is_switched_on(uint16_t sw) { return (sw & 0x006F) == SW_SWITCHED_ON; }
    
    bool is_operation_enabled(uint16_t sw) { return (sw & 0x006F) == SW_OPERATION_ENABLED; } 
    
    bool is_switch_on_disabled(uint16_t sw) { return (sw & 0x004F) == SW_SWITCH_ON_DISABLED; }
    
    bool is_fault(uint16_t sw) { return (sw & SW_FAULT); }
    
    bool is_setpoint_acknowledge(uint16_t sw) { return (sw & 0x1000) == SW_SETPOINT_ACKNOWLEDGE; }
};

} //namespace micros
#endif // DRIVER_MINAS_DRIVER_HPP_