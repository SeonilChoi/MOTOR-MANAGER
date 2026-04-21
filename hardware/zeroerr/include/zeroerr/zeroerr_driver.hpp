#ifndef ZEROERR_ZEROERR_DRIVER_HPP_
#define ZEROERR_ZEROERR_DRIVER_HPP_

#include <string>

#include "motor_interface/motor_driver.hpp"

namespace zeroerr {

inline constexpr uint8_t ID_MIN_POSITION_LIMIT = 50;
inline constexpr uint8_t ID_MAX_POSITION_LIMIT = 51;
inline constexpr uint8_t ID_PROFILE_VELOCITY = 52;
inline constexpr uint8_t ID_PROFILE_ACCELERATION = 53;
inline constexpr uint8_t ID_PROFILE_DECELERATION = 54;
inline constexpr uint8_t ID_RXPDO = 98;
inline constexpr uint8_t ID_TXPDO = 99;

class ZeroerrDriver : public motor_interface::MotorDriver {
public:
    explicit ZeroerrDriver(const motor_interface::driver_config_t& config);

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

} // namespace zeroerr

#endif // ZEROERR_ZEROERR_DRIVER_HPP_