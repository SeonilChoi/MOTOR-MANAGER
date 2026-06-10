#ifndef MOTOR_MANAGER_MOTOR_MANAGER_HPP_
#define MOTOR_MANAGER_MOTOR_MANAGER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "motor_interface/motor_master.hpp"
#include "motor_interface/motor_driver.hpp"
#include "motor_interface/motor_controller.hpp"

namespace motor_manager {

inline constexpr uint64_t NSEC_PER_SEC = 1000000000;

inline constexpr uint8_t MAX_MASTER_SIZE = 8;
inline constexpr uint8_t MAX_DRIVER_SIZE = 8;
inline constexpr uint8_t MAX_CONTROLLER_SIZE = 16;

enum class CommunicationType {
    Ethercat,
    Canopen,
    Dynamixel
};

enum class DriverType {
    Minas,
    Zeroerr,
    Dynamixel
};

inline CommunicationType toCommunicationType(const std::string& type) {
    if (type == "ethercat") return CommunicationType::Ethercat;
    if (type == "canopen") return CommunicationType::Canopen;
    if (type == "dynamixel") return CommunicationType::Dynamixel;
    throw std::runtime_error("Invalid communication type.");
}

inline DriverType toDriverType(const std::string& type) {
    if (type == "minas") return DriverType::Minas;
    if (type == "zeroerr") return DriverType::Zeroerr;
    if (type == "dynamixel") return DriverType::Dynamixel;
    throw std::runtime_error("Invalid driver type.");
}

class MotorManager {
public:
    explicit MotorManager(const std::string& config_file);

    virtual ~MotorManager() = default;

    void run();

    void write(const motor_interface::motor_frame_t* command, const uint8_t size);

    void read(motor_interface::motor_frame_t* status);

    /** `user_command` / Empty: start CiA402 disable until all axes report disabled, then `run()` returns. */
    void request_stop();

    /** Node teardown: clear the RT loop flag so `run()` exits (after current sleep slice); does not wait for drive disable. */
    void request_exit();

    uint32_t period() const { return period_; }

    uint8_t number_of_controllers() const { return number_of_controllers_; }

private:
    void loadConfigurations(const std::string& config_file);

    void initialize();

    void start() { for (auto& m_iter : masters_) m_iter.second->activate(); }

    void stop() { for (auto& m_iter : masters_) m_iter.second->deactivate(); }

    void enable();

    void disable();

    void check(const motor_interface::motor_frame_t* status);

    void update();

    std::unordered_map<uint8_t, std::unique_ptr<motor_interface::MotorMaster>> masters_;

    std::unordered_map<uint8_t, std::unique_ptr<motor_interface::MotorDriver>> drivers_;

    std::unique_ptr<motor_interface::MotorController> controllers_[MAX_CONTROLLER_SIZE];

    uint32_t period_{0};

    uint8_t number_of_controllers_{0};

    uint32_t frequency_{0};

    bool is_enable_{false};

    bool is_disabled_{false};

    std::atomic<bool> on_disabled_{false};

    std::atomic<bool> running_{true};

    std::mutex frame_mutex_;

    std::atomic<bool> is_command_changed_{false};

    motor_interface::motor_frame_t command_[MAX_CONTROLLER_SIZE];

    motor_interface::motor_frame_t status_[MAX_CONTROLLER_SIZE];
};

} // namespace motor_manager
#endif // MOTOR_MANAGER_MOTOR_MANAGER_HPP_
