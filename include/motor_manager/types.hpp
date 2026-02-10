#ifndef MOTOR_MANAGER_TYPES_HPP_
#define MOTOR_MANAGER_TYPES_HPP_

#include <string>
#include <cstdint>
#include <cstddef>
#include <stdexcept>
#include <type_traits>

namespace micros {

inline constexpr std::size_t MAX_DATA_SIZE  = 4;  // Maximum size of data array (32-bit).
inline constexpr uint8_t MAX_ITEM_SIZE      = 32; // Maximum number of configuration entries per driver.
inline constexpr uint8_t MAX_MASTER_SIZE    = 8;  // Maximum number of masters.

// Interface ID to identify each entries.
inline constexpr uint8_t ID_CONTROLWORD      = 0;
inline constexpr uint8_t ID_TARGET_POSITION  = 1;
inline constexpr uint8_t ID_TARGET_VELOCITY  = 2;
inline constexpr uint8_t ID_TARGET_TORQUE    = 3;
inline constexpr uint8_t ID_STATUSWORD       = 4;
inline constexpr uint8_t ID_ERRORCODE        = 5;
inline constexpr uint8_t ID_CURRENT_POSITION = 6;
inline constexpr uint8_t ID_CURRENT_VELOCITY = 7;
inline constexpr uint8_t ID_CURRENT_TORQUE   = 8;

// Communication type of a master.
enum class CommunicationType {
    Ethercat, // EtherCAT
    Canopen,  // CANopen
    Dynamixel // Dynamixel
};

// Driver type of a driver.
enum class DriverType {
    Minas,    // Minas
    Zeroerr,  // ZeroErr (e.g. eRob)
    Dynamxiel // Dynamixel 
};

// Data type of a value.
enum class ValueType { 
    U8,  // Unsigned 8-bit integer
    U16, // Unsigned 16-bit integer (e.g. controlword)
    U32, // Unsigned 32-bit integer
    S8,  // Signed 8-bit integer
    S16, // Signed 16-bit integer
    S32  // Signed 32-bit integer
};

// State of a driver based on CiA.
enum class DriverState {
    SwitchOnDisabled, // "Switch on disabled"
    ReadyToSwitchOn,  // "Ready to switch on"
    SwitchedOn,       // "Switched on"
    OperationEnabled, // "Operation enabled"
};

/**************************************************************************************************/

struct master_config_t {
    uint8_t id;                // Master ID
    uint8_t number_of_slaves;  // Number of slaves in a master.
    unsigned int master_idx{}; // Master index for EtherCAT.
};

struct slave_config_t {
    uint8_t master_id;   // ID of the master that owns this slave.
    uint8_t driver_id;   // ID of the driver associated with this slave.
    uint16_t alias{};    // EtherCAT slave alias.
    uint16_t position{}; // EtherCAT slave position, used with alias.
    uint32_t vid{};      // Vendor ID
    uint32_t pid{};      // Product ID
};

struct driver_config_t {
    uint8_t id;                    // Driver ID
    uint32_t pulse_per_revolution; // Encoder counts per revolution (pulses).
    double rated_torque;           // Rated torque [Nm].
    double unit_torque;            // Torque scale: 1 unit = (unit torque)% of rated torque.
    double lower;                  // Minimum position limit [pulses].
    double upper;                  // Maximum position limit [pulses].
    double speed;                  // Max motor speed [rpm].
    double acceleration;           // Max motor acceleration [rad/s^2].
    double deceleration;           // Max motor deceleration [rad/s^2].
    double profile_velocity;       // Profile velocity [rad/s].
    double profile_acceleration;   // Profile acceleration [rad/s^2].
    double profile_deceleration;   // Profile deceleration [rad/s^2].
};

struct entry_table_t {
    uint8_t id;                     // Table entry ID or Interface ID.
    uint16_t index;                 // Index of the object to configure.
    uint8_t subindex;               // Subindex of the object to configure.
    ValueType type;                 // Object value type.
    std::size_t size;               // Size of the value [bytes].
    uint8_t data[MAX_DATA_SIZE]{0}; // Value encoded as bytes.
};

/**************************************************************************************************/

inline CommunicationType to_communication_type(const std::string& comm_type) {
    // Converts a config string (e.g., "ethercat") to CommunicationType.
    if (comm_type == "ethercat") return CommunicationType::Ethercat;
    if (comm_type == "canopen")  return CommunicationType::Canopen;
    if (comm_type == "dynamixel") return CommunicationType::Dynamixel;
    throw std::runtime_error("Unsupported communication type: " + comm_type);
}

inline DriverType to_driver_type(const std::string& driver_type) {
    // Converts a config file (e.g., "minas") to DriverType.
    if (driver_type == "minas") return DriverType::Minas;
    if (driver_type == "zeroerr") return DriverType::Zeroerr;
    if (driver_type == "dynamixel") return DriverType::Dynamxiel;
    throw std::runtime_error("Unsupported driver type: " + driver_type);
}

inline ValueType to_value_type(const std::string& type) {
    // Converts a config file (e.g., "u8") to ValueType.
    if (type == "u8") return ValueType::U8;
    if (type == "u16") return ValueType::U16;
    if (type == "u32") return ValueType::U32;
    if (type == "s8") return ValueType::S8;
    if (type == "s16") return ValueType::S16;
    if (type == "s32") return ValueType::S32;
    throw std::runtime_error("Unsupported data type: " + type);
}

template <typename T>
inline T to_value(const std::array<uint8_t, MAX_DATA_SIZE>& data) {
    // Decodes little-endian bytes into an integral value T.
    using U = std::make_unsigned_t<T>;
    U u = 0;
    for (std::size_t i = 0; i < sizeof(T); ++i) {
        u |= (static_cast<U>(data[i]) << (i * 8));
    }
    return static_cast<T>(u);
}

template <typename T>
inline void fill_data(const T& value, std::array<uint8_t, MAX_DATA_SIZE>& data) {
    // Encodes an integral value T into little-endian bytes.
    using U = std::make_unsigned_t<T>;
    U u = static_cast<U>(value);
    for (std::size_t i = 0; i < sizeof(T); ++i) {
        data[i] = static_cast<uint8_t>(
            (u >> (8 * i)) & static_cast<U>(0xFF)
        );
    }
}

} // namespace micros
#endif // #ifndef MOTOR_MANAGER_TYPES_HPP_