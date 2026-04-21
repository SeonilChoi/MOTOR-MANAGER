#ifndef MINAS_MINAS_DRIVER_HPP_
#define MINAS_MINAS_DRIVER_HPP_

#include <string>

#include "motor_interface/motor_driver.hpp"

namespace minas {

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

class MinasDriver : public motor_interface::MotorDriver {
public:
    explicit MinasDriver(const motor_interface::driver_config_t& config);

    void loadParameters(const std::string& param_file) override;

    bool isEnabled(const uint8_t* data, motor_interface::DriverState& driver_state, uint8_t* out) override;

    bool isDisabled(const uint8_t* data, motor_interface::DriverState& driver_state, uint8_t* out) override;

    bool isReceived(const uint8_t* data, uint8_t* out) override;

    double position(const int32_t value) override;

    double velocity(const int32_t value) override;

    double torque(const int16_t value) override;

    int32_t position(const double value) override;

    int32_t velocity(const double value) override;

    int16_t torque(const double value) override;
};

} // namespace minas

#endif // MINAS_MINAS_DRIVER_HPP_
