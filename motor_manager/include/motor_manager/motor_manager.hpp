#ifndef MOTOR_MANAGER_MOTOR_MANAGER_HPP_
#define MOTOR_MANAGER_MOTOR_MANAGER_HPP_

#include <atomic>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "motor_interface/motor_master.hpp"
#include "motor_interface/motor_driver.hpp"
#include "motor_interface/motor_controller.hpp"

namespace motor_manager {

inline constexpr uint64_t NSEC_PER_SEC = 1000000000;

enum class CommunicationType {
    Ethercat,
    Canopen,
    SocketCAN,
    Serial
};

enum class DriverType {
    Minas,
    Zeroerr,
    Dynamixel,
    CubeMars,
    Unitree
};

inline CommunicationType toCommunicationType(const std::string& type) {
    if (type == "ethercat") return CommunicationType::Ethercat;
    if (type == "canopen") return CommunicationType::Canopen;
    if (type == "socketcan") return CommunicationType::SocketCAN;
    if (type == "serial") return CommunicationType::Serial;
    if (type == "dynamixel") return CommunicationType::Serial;
    throw std::runtime_error("Invalid communication type.");
}

inline DriverType toDriverType(const std::string& type) {
    if (type == "minas") return DriverType::Minas;
    if (type == "zeroerr") return DriverType::Zeroerr;
    if (type == "dynamixel") return DriverType::Dynamixel;
    if (type == "cubemars") return DriverType::CubeMars;
    throw std::runtime_error("Invalid driver type.");
}

class MotorManager {
public:
    explicit MotorManager(const std::string& config_file);

    virtual ~MotorManager();

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

    void startSerialWorkers();

    void stopSerialWorkers();

    void serialWorker(uint8_t master_id);

    bool isSerialMaster(uint8_t master_id) const;

    bool shouldServiceController(
        uint8_t controller_index,
        std::optional<uint8_t> serial_master_id) const;

    void enableControllers(std::optional<uint8_t> serial_master_id);

    void disableControllers(std::optional<uint8_t> serial_master_id);

    void updateControllers(std::optional<uint8_t> serial_master_id);

    void refreshEnabled();

    void refreshDisabled();

    void setSerialException(std::exception_ptr exception);

    void rethrowSerialExceptionIfAny();

    std::unordered_map<uint8_t, std::unique_ptr<motor_interface::MotorMaster>> masters_;

    std::unordered_map<uint8_t, std::unique_ptr<motor_interface::MotorDriver>> drivers_;

    std::unordered_set<uint8_t> serial_master_ids_;

    std::unique_ptr<motor_interface::MotorController> controllers_[motor_interface::MAX_CONTROLLER_SIZE];

    uint32_t period_{0};

    uint8_t number_of_controllers_{0};

    uint32_t frequency_{0};

    std::atomic<bool> is_enable_{false};

    std::atomic<bool> is_disabled_{false};

    std::atomic<bool> controller_enabled_[motor_interface::MAX_CONTROLLER_SIZE]{};

    std::atomic<bool> controller_disabled_[motor_interface::MAX_CONTROLLER_SIZE]{};

    std::atomic<bool> on_disabled_{false};

    std::atomic<bool> running_{true};

    std::atomic<bool> serial_running_{false};

    std::mutex frame_mutex_;

    std::mutex serial_exception_mutex_;

    std::exception_ptr serial_exception_{nullptr};

    std::vector<std::thread> serial_threads_;

    motor_interface::motor_frame_t command_[motor_interface::MAX_CONTROLLER_SIZE];

    motor_interface::motor_frame_t status_[motor_interface::MAX_CONTROLLER_SIZE];

    bool command_dirty_[motor_interface::MAX_CONTROLLER_SIZE]{};

    uint64_t command_sequence_[motor_interface::MAX_CONTROLLER_SIZE]{};

    uint64_t applied_command_sequence_[motor_interface::MAX_CONTROLLER_SIZE]{};
};

} // namespace motor_manager
#endif // MOTOR_MANAGER_MOTOR_MANAGER_HPP_
