#ifndef MOTOR_INTERFACE_MOTOR_DRIVER_HPP_
#define MOTOR_INTERFACE_MOTOR_DRIVER_HPP_

#include <string>
#include <cstdint>
#include <cstddef>
#include <stdexcept>
#include <type_traits>

#include "common_motor_interface/motor_frame.hpp"

namespace motor_interface {

inline constexpr uint8_t MAX_DATA_SIZE = 4;
inline constexpr uint8_t MAX_ITEM_SIZE = 32;

inline constexpr uint8_t ID_CONTROLWORD      = 0;
inline constexpr uint8_t ID_TARGET_POSITION  = 1;
inline constexpr uint8_t ID_TARGET_VELOCITY  = 2;
inline constexpr uint8_t ID_TARGET_TORQUE    = 3;
inline constexpr uint8_t ID_STATUSWORD       = 4;
inline constexpr uint8_t ID_ERRORCODE        = 5;
inline constexpr uint8_t ID_CURRENT_POSITION = 6;
inline constexpr uint8_t ID_CURRENT_VELOCITY = 7;
inline constexpr uint8_t ID_CURRENT_TORQUE   = 8;

enum class DataType {
    U8,
    U16,
    U32,
    U64,
    S8,
    S16,
    S32
};

enum class DriverState {
    Fault,
    SwitchOnDisabled,
    ReadyToSwitchOn,
    SwitchedOn,
    OperationEnabled
};

struct driver_config_t {
    uint8_t id;
    uint32_t pulse_per_revolution;
    double rated_torque;
    double unit_torque;
    double lower;
    double upper;
    double speed;
    double acceleration;
    double deceleration;
    double profile_velocity;
    double profile_acceleration;
    double profile_deceleration;
};

struct entry_table_t {
    uint8_t id;
    uint16_t index;
    uint8_t subindex;

    DataType type;
    uint8_t size;
    uint8_t data[MAX_DATA_SIZE];
};

inline DataType toDataType(const std::string& type) {
    if (type == "u8") return DataType::U8;
    if (type == "u16") return DataType::U16;
    if (type == "u32") return DataType::U32;
    if (type == "u64") return DataType::U64;
    if (type == "s8") return DataType::S8;
    if (type == "s16") return DataType::S16;
    if (type == "s32") return DataType::S32;
    throw std::runtime_error("Invalid data type.");
}

template <typename T>
inline T value(const uint8_t* data) {
    using U = std::make_unsigned_t<T>;
    U u = 0;
    for (std::size_t i = 0; i < sizeof(T); ++i) {
        u |= (static_cast<U>(data[i]) << (i * 8));
    }
    return static_cast<T>(u);
}

template <typename T>
inline void fill(const T& value, uint8_t* data) {
    using U = std::make_unsigned_t<T>;
    U u = static_cast<U>(value);
    for (std::size_t i = 0; i < sizeof(T); ++i) {
        data[i] = static_cast<uint8_t>(
            (u >> (i * 8)) & static_cast<U>(0xFF)
        );
    }
}

class MotorDriver {
public:
    explicit MotorDriver(const driver_config_t& config)
    : config_(config) {}

    virtual ~MotorDriver() = default;

    virtual void loadParameters(const std::string& param_file) = 0;

    virtual bool isEnabled(const uint8_t* data, DriverState& driver_state, uint8_t* out) = 0;

    virtual bool isDisabled(const uint8_t* data, DriverState& driver_state, uint8_t* out) = 0;

    virtual bool isReceived(const uint8_t* data, uint8_t* out) = 0;

    virtual double position(const int32_t value) = 0;

    virtual double velocity(const int32_t value) = 0;

    virtual double torque(const int16_t value) = 0;

    virtual int32_t position(const double value) = 0;

    virtual int32_t velocity(const double value) = 0;

    virtual int16_t torque(const double value) = 0;

    const entry_table_t* items() const { return items_; }

    const entry_table_t* interfaces() const { return interfaces_; }

    uint8_t number_of_items() const { return number_of_items_; }

    uint8_t number_of_interfaces() const { return number_of_interfaces_; }

    uint8_t number_of_rx_interfaces() const { return number_of_rx_interfaces_; }

    uint8_t number_of_tx_interfaces() const { return number_of_tx_interfaces_; }

protected:
    entry_table_t items_[MAX_ITEM_SIZE];

    entry_table_t interfaces_[MAX_INTERFACE_SIZE];

    uint8_t number_of_items_{0};

    uint8_t number_of_interfaces_{0};

    uint8_t number_of_rx_interfaces_{0};

    uint8_t number_of_tx_interfaces_{0};

    const driver_config_t config_;
};

} // namespace motor_interface
#endif // MOTOR_INTERFACE_MOTOR_DRIVER_HPP_
