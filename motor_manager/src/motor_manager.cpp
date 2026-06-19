#include <algorithm>
#include <chrono>
#include <cstring>
#include <filesystem>

#include <time.h>
#include <sched.h>
#include <errno.h>
#include <unistd.h>
#include <sys/mman.h>
#include <yaml-cpp/yaml.h>

#include "motor_manager/motor_manager.hpp"
#include "ethercat/ethercat_master.hpp"
#include "ethercat/ethercat_controller.hpp"
#include "canopen/canopen_master.hpp"
#include "canopen/canopen_controller.hpp"
#include "serial/serial_master.hpp"
#include "serial/serial_controller.hpp"
#include "socketcan/socketcan_master.hpp"
#include "socketcan/socketcan_controller.hpp"

#include "minas/minas_driver.hpp"
#include "zeroerr/zeroerr_driver.hpp"
#include "dynamixel/dynamixel_driver.hpp"
#include "cubemars/cubemars_driver.hpp"

namespace {

void unlock_memory()
{
    (void)munlockall();
}

void stack_prefault()
{
    unsigned char dummy[8 * 1024];
    std::memset(dummy, 0, sizeof(dummy));
}

bool isYamlFilePath(const std::filesystem::path& path)
{
    const std::string ext = path.extension().string();
    return ext == ".yaml" || ext == ".yml";
}

std::filesystem::path resolveConfigRelativePath(
    const std::string& config_file,
    std::filesystem::path path)
{
    if (path.is_absolute()) return path.lexically_normal();

    return (std::filesystem::path(config_file).parent_path() / path).lexically_normal();
}

std::string resolveDriverParamPath(
    const YAML::Node& driver_node,
    const std::string& config_file)
{
    const std::string type = driver_node["type"].as<std::string>();
    std::filesystem::path param_path =
        resolveConfigRelativePath(config_file, driver_node["param_file"].as<std::string>());

    if (!isYamlFilePath(param_path)) {
        param_path /= type + ".yaml";
    }

    return param_path.lexically_normal().string();
}

uint8_t readControllerIndex(const YAML::Node& slave_node)
{
    const int controller_index = slave_node["controller_index"].as<int>();
    if (controller_index < 0 ||
        controller_index >= static_cast<int>(motor_interface::MAX_CONTROLLER_SIZE)) {
        throw std::runtime_error(
            "controller_index must be in range 0.." +
            std::to_string(motor_interface::MAX_CONTROLLER_SIZE - 1) + ".");
    }
    return static_cast<uint8_t>(controller_index);
}

uint8_t readCanId(const YAML::Node& slave_node)
{
    const YAML::Node can_id = slave_node["can_id"] ? slave_node["can_id"] : slave_node["node_id"];
    if (!can_id) throw std::runtime_error("slave requires can_id.");

    const int id = can_id.as<int>();
    if (id < 0 || id > 255) {
        throw std::runtime_error("can_id must be in range 0..255.");
    }

    return static_cast<uint8_t>(id);
}

uint8_t readProfileMode(const YAML::Node& slave_node, bool required = true)
{
    const YAML::Node profile_mode_node = slave_node["profile_mode"];
    if (!profile_mode_node) {
        if (required) throw std::runtime_error("slave requires profile_mode.");
        return 0;
    }

    const int profile_mode = profile_mode_node.as<int>();
    if (profile_mode < 0 || profile_mode > 2) {
        throw std::runtime_error("profile_mode must be 0, 1, or 2.");
    }

    return static_cast<uint8_t>(profile_mode);
}

} // namespace

motor_manager::MotorManager::MotorManager(const std::string& config_file)
{
    loadConfigurations(config_file);

    initialize();
}

motor_manager::MotorManager::~MotorManager()
{
    request_exit();
    stopSerialWorkers();
}

void motor_manager::MotorManager::loadConfigurations(const std::string& config_file)
{
    YAML::Node root = YAML::LoadFile(config_file);
    if (!root) throw std::runtime_error("Failed to load configuration file.");

    period_ = root["period"].as<uint32_t>();
    if (period_ == 0) throw std::runtime_error("Motor manager period must be greater than zero.");

    YAML::Node masters = root["masters"];
    if (!masters || !masters.IsSequence()) throw std::runtime_error("Invalid masters configuration.");

    uint8_t s_idx{0};
    masters_.reserve(motor_interface::MAX_MASTER_SIZE);
    for (const auto& m : masters) {
        motor_interface::master_config_t m_cfg{};
        m_cfg.id = m["id"].as<uint8_t>();
        m_cfg.number_of_slaves = m["number_of_slaves"].as<uint8_t>();

        YAML::Node slaves = m["slaves"];
        if (!slaves || !slaves.IsSequence()) throw std::runtime_error("Invalid slaves configuration.");

        switch (toCommunicationType(m["type"].as<std::string>())) {
        case CommunicationType::Ethercat: {
            m_cfg.ethercat_master_index = m["ethercat_master_index"].as<unsigned int>();
            masters_[m_cfg.id] = std::make_unique<ethercat::EthercatMaster>(m_cfg);

            for (uint8_t i = 0; i < m["number_of_slaves"].as<uint8_t>(); ++i) {
                motor_interface::slave_config_t s_cfg{};
                s_cfg.controller_index = readControllerIndex(slaves[i]);
                s_cfg.master_id = m_cfg.id;
                s_cfg.driver_id = slaves[i]["driver_id"].as<uint8_t>();
                s_cfg.alias = slaves[i]["alias"].as<uint16_t>();
                s_cfg.position = slaves[i]["position"].as<uint16_t>();
                s_cfg.vendor_id = slaves[i]["vendor_id"].as<uint32_t>();
                s_cfg.product_id = slaves[i]["product_id"].as<uint32_t>();
                s_cfg.profile_mode = readProfileMode(slaves[i]);

                controllers_[s_cfg.controller_index] = std::make_unique<ethercat::EthercatController>(s_cfg);
                s_idx++;
            }
            break;
        } case CommunicationType::Canopen: {
            m_cfg.can_interface_index = m["can_interface_index"].as<unsigned int>();
            m_cfg.can_bitrate = m["can_bitrate"].as<unsigned int>();

            masters_[m_cfg.id] = std::make_unique<canopen::CanopenMaster>(m_cfg);

            for (uint8_t i = 0; i < m["number_of_slaves"].as<uint8_t>(); ++i) {
                motor_interface::slave_config_t s_cfg{};
                s_cfg.controller_index = readControllerIndex(slaves[i]);
                s_cfg.master_id = m_cfg.id;
                s_cfg.driver_id = slaves[i]["driver_id"].as<uint8_t>();
                s_cfg.can_id = readCanId(slaves[i]);
                s_cfg.profile_mode = readProfileMode(slaves[i]);

                controllers_[s_cfg.controller_index] = std::make_unique<canopen::CanopenController>(s_cfg);
                s_idx++;
            }
            break;
        } case CommunicationType::Serial: {
            if (m["serial_port"]) {
                m_cfg.serial_port = m["serial_port"].as<std::string>();
            } else if (m["port"]) {
                m_cfg.serial_port = m["port"].as<std::string>();
            } else {
                throw std::runtime_error("Serial master requires serial_port.");
            }

            if (m["serial_baudrate"]) {
                m_cfg.serial_baudrate = m["serial_baudrate"].as<unsigned int>();
            } else if (m["baudrate"]) {
                m_cfg.serial_baudrate = m["baudrate"].as<unsigned int>();
            } else {
                m_cfg.serial_baudrate = 57600;
            }

            serial_master_ids_.insert(m_cfg.id);
            masters_[m_cfg.id] = std::make_unique<serial::SerialMaster>(m_cfg);

            for (uint8_t i = 0; i < m["number_of_slaves"].as<uint8_t>(); ++i) {
                motor_interface::slave_config_t s_cfg{};
                s_cfg.controller_index = readControllerIndex(slaves[i]);
                s_cfg.master_id = m_cfg.id;
                s_cfg.driver_id = slaves[i]["driver_id"].as<uint8_t>();
                s_cfg.can_id = readCanId(slaves[i]);
                s_cfg.profile_mode = readProfileMode(slaves[i]);

                controllers_[s_cfg.controller_index] = std::make_unique<serial::SerialController>(s_cfg);
                s_idx++;
            }
            break;
        } case CommunicationType::SocketCAN: {
            m_cfg.can_interface_index = m["can_interface_index"].as<unsigned int>();
            m_cfg.can_bitrate = m["can_bitrate"] ? m["can_bitrate"].as<unsigned int>() : 0;

            masters_[m_cfg.id] = std::make_unique<socketcan::SocketcanMaster>(m_cfg);

            for (uint8_t i = 0; i < m["number_of_slaves"].as<uint8_t>(); ++i) {
                motor_interface::slave_config_t s_cfg{};
                s_cfg.controller_index = readControllerIndex(slaves[i]);
                s_cfg.master_id = m_cfg.id;
                s_cfg.driver_id = slaves[i]["driver_id"].as<uint8_t>();
                s_cfg.can_id = readCanId(slaves[i]);
                s_cfg.profile_mode = readProfileMode(slaves[i], false);

                controllers_[s_cfg.controller_index] = std::make_unique<socketcan::SocketcanController>(s_cfg);
                s_idx++;
            }
            break;
        } default: {
            throw std::runtime_error("Invalid communication type.");
        }
        }
    }
    number_of_controllers_ = s_idx;

    YAML::Node drivers = root["drivers"];
    if (!drivers || !drivers.IsSequence()) throw std::runtime_error("Invalid drivers configuration.");

    drivers_.reserve(motor_interface::MAX_DRIVER_SIZE);
    for (const auto& d : drivers) {
        motor_interface::driver_config_t d_cfg{};
        d_cfg.id = d["id"].as<uint8_t>();
        d_cfg.pulse_per_revolution = d["pulse_per_revolution"].as<uint32_t>();
        d_cfg.rated_effort = d["rated_effort"].as<double>();
        d_cfg.unit_effort = d["unit_effort"].as<double>();
        d_cfg.lower = d["lower"].as<double>();
        d_cfg.upper = d["upper"].as<double>();
        d_cfg.speed = d["speed"].as<double>();
        d_cfg.acceleration = d["acceleration"].as<double>();
        d_cfg.deceleration = d["deceleration"].as<double>();
        d_cfg.profile_velocity = d["profile_velocity"].as<double>();
        d_cfg.profile_acceleration = d["profile_acceleration"].as<double>();
        d_cfg.profile_deceleration = d["profile_deceleration"].as<double>();
        d_cfg.profile_position_value = d["profile_position_value"].as<int8_t>();
        d_cfg.profile_velocity_value = d["profile_velocity_value"].as<int8_t>();
        d_cfg.profile_effort_value = d["profile_effort_value"].as<int8_t>();

        switch (toDriverType(d["type"].as<std::string>())) {
        case DriverType::Minas: {
            drivers_[d_cfg.id] = std::make_unique<minas::MinasDriver>(d_cfg);
            break;
        } case DriverType::Zeroerr: {
            drivers_[d_cfg.id] = std::make_unique<zeroerr::ZeroerrDriver>(d_cfg);
            break;
        } case DriverType::Dynamixel: {
            drivers_[d_cfg.id] = std::make_unique<dynamixel::DynamixelDriver>(d_cfg);
            break;
        } case DriverType::CubeMars: {
            drivers_[d_cfg.id] = std::make_unique<cubemars::CubemarsDriver>(d_cfg);
            break;
        } default: {
            throw std::runtime_error("Invalid driver type.");
        }
        }
        const std::string param_path = resolveDriverParamPath(d, config_file);
        drivers_.at(d_cfg.id)->loadParameters(param_path);
    }
}

void motor_manager::MotorManager::initialize()
{
    frequency_ = NSEC_PER_SEC / period_;

    for (auto& m_iter : masters_) m_iter.second->initialize();

    for (uint8_t i = 0; i < number_of_controllers_; ++i) {
        uint8_t m_id = controllers_[i]->master_id();
        uint8_t d_id = controllers_[i]->driver_id();
        controllers_[i]->initialize(*masters_.at(m_id), *drivers_.at(d_id));
    }
}

void motor_manager::MotorManager::startSerialWorkers()
{
    stopSerialWorkers();

    {
        std::lock_guard<std::mutex> lock(serial_exception_mutex_);
        serial_exception_ = nullptr;
    }

    if (serial_master_ids_.empty()) return;

    serial_running_.store(true, std::memory_order_release);
    serial_threads_.reserve(serial_master_ids_.size());
    for (uint8_t master_id : serial_master_ids_) {
        serial_threads_.emplace_back(&MotorManager::serialWorker, this, master_id);
    }
}

void motor_manager::MotorManager::stopSerialWorkers()
{
    serial_running_.store(false, std::memory_order_release);

    for (std::thread& thread : serial_threads_) {
        if (thread.joinable()) thread.join();
    }
    serial_threads_.clear();
}

void motor_manager::MotorManager::serialWorker(uint8_t master_id)
{
    try {
        auto master_iter = masters_.find(master_id);
        if (master_iter == masters_.end()) {
            throw std::runtime_error("Serial worker master does not exist.");
        }

        const auto cycle = std::chrono::nanoseconds(period_);
        auto next_wakeup = std::chrono::steady_clock::now();

        while (serial_running_.load(std::memory_order_acquire) &&
               running_.load(std::memory_order_acquire))
        {
            next_wakeup += cycle;
            std::this_thread::sleep_until(next_wakeup);

            if (!serial_running_.load(std::memory_order_acquire) ||
                !running_.load(std::memory_order_acquire))
            {
                break;
            }

            motor_interface::MotorMaster& master = *master_iter->second;
            master.receive();

            if (on_disabled_.load(std::memory_order_acquire)) {
                disableControllers(master_id);
            } else {
                enableControllers(master_id);
                updateControllers(master_id);
            }

            master.transmit();

            const auto now = std::chrono::steady_clock::now();
            if (next_wakeup < now - cycle) {
                next_wakeup = now;
            }
        }
    } catch (...) {
        setSerialException(std::current_exception());
        serial_running_.store(false, std::memory_order_release);
        running_.store(false, std::memory_order_release);
    }
}

bool motor_manager::MotorManager::isSerialMaster(uint8_t master_id) const
{
    return serial_master_ids_.find(master_id) != serial_master_ids_.end();
}

bool motor_manager::MotorManager::shouldServiceController(
    uint8_t controller_index,
    std::optional<uint8_t> serial_master_id) const
{
    if (controller_index >= number_of_controllers_ || !controllers_[controller_index]) {
        return false;
    }

    const uint8_t master_id = controllers_[controller_index]->master_id();
    if (serial_master_id) {
        return master_id == *serial_master_id;
    }

    return !isSerialMaster(master_id);
}

void motor_manager::MotorManager::enableControllers(std::optional<uint8_t> serial_master_id)
{
    for (uint8_t i = 0; i < number_of_controllers_; ++i) {
        if (!shouldServiceController(i, serial_master_id)) continue;

        if (!controller_enabled_[i].load(std::memory_order_acquire)) {
            controller_enabled_[i].store(controllers_[i]->enable(), std::memory_order_release);
        }
    }

    refreshEnabled();
}

void motor_manager::MotorManager::disableControllers(std::optional<uint8_t> serial_master_id)
{
    for (uint8_t i = 0; i < number_of_controllers_; ++i) {
        if (!shouldServiceController(i, serial_master_id)) continue;

        if (!controller_disabled_[i].load(std::memory_order_acquire)) {
            controller_disabled_[i].store(controllers_[i]->disable(), std::memory_order_release);
        }
    }

    refreshDisabled();
}

void motor_manager::MotorManager::write(const motor_interface::motor_frame_t* command, const uint8_t size)
{
    std::lock_guard<std::mutex> lock(frame_mutex_);
    const uint8_t n = std::min(size, motor_interface::MAX_CONTROLLER_SIZE);
    for (uint8_t i = 0; i < n; ++i) {
        const uint8_t controller_index = command[i].controller_index;
        if (controller_index >= number_of_controllers_) continue;

        command_[controller_index] = command[i];
        ++command_sequence_[controller_index];
        command_dirty_[controller_index] = true;
    }
}

void motor_manager::MotorManager::read(motor_interface::motor_frame_t* status)
{
    std::lock_guard<std::mutex> lock(frame_mutex_);
    for (uint8_t i = 0; i < number_of_controllers_; ++i) {
        status[i] = status_[i];
    }
}

void motor_manager::MotorManager::request_stop()
{
    on_disabled_.store(true, std::memory_order_release);
}

void motor_manager::MotorManager::request_exit()
{
    running_.store(false, std::memory_order_release);
    serial_running_.store(false, std::memory_order_release);
}

void motor_manager::MotorManager::updateControllers(std::optional<uint8_t> serial_master_id)
{
    for (uint8_t i = 0; i < number_of_controllers_; ++i) {
        if (!shouldServiceController(i, serial_master_id)) continue;
        if (!controller_enabled_[i].load(std::memory_order_acquire)) continue;

        motor_interface::motor_frame_t controller_status{};
        controllers_[i]->read(controller_status);

        motor_interface::motor_frame_t pending_command{};
        uint64_t pending_sequence{0};
        bool has_pending_command{false};

        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            status_[i] = controller_status;

            if (controller_status.errorcode == 0 &&
                command_sequence_[i] != applied_command_sequence_[i])
            {
                pending_command = command_[i];
                pending_sequence = command_sequence_[i];
                has_pending_command = true;
            }
        }

        if (has_pending_command) {
            controllers_[i]->write(pending_command);

            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (applied_command_sequence_[i] < pending_sequence) {
                applied_command_sequence_[i] = pending_sequence;
            }
            command_dirty_[i] = command_sequence_[i] != applied_command_sequence_[i];
        } else {
            controllers_[i]->check(controller_status);
        }
    }
}

void motor_manager::MotorManager::refreshEnabled()
{
    bool all_enabled = true;
    for (uint8_t i = 0; i < number_of_controllers_; ++i) {
        all_enabled = all_enabled && controller_enabled_[i].load(std::memory_order_acquire);
    }
    is_enable_.store(all_enabled, std::memory_order_release);
}

void motor_manager::MotorManager::refreshDisabled()
{
    bool all_disabled = true;
    for (uint8_t i = 0; i < number_of_controllers_; ++i) {
        all_disabled = all_disabled && controller_disabled_[i].load(std::memory_order_acquire);
    }
    is_disabled_.store(all_disabled, std::memory_order_release);
}

void motor_manager::MotorManager::setSerialException(std::exception_ptr exception)
{
    std::lock_guard<std::mutex> lock(serial_exception_mutex_);
    if (!serial_exception_) serial_exception_ = exception;
}

void motor_manager::MotorManager::rethrowSerialExceptionIfAny()
{
    std::exception_ptr exception;
    {
        std::lock_guard<std::mutex> lock(serial_exception_mutex_);
        exception = serial_exception_;
    }

    if (exception) std::rethrow_exception(exception);
}

void motor_manager::MotorManager::run()
{
    running_.store(true, std::memory_order_release);
    start();
    bool memory_locked{false};

    try {
        startSerialWorkers();

        if (sysconf(_SC_PAGESIZE) == -1) {
            throw std::runtime_error("sysconf(_SC_PAGESIZE) failed.");
        }

        if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
            throw std::runtime_error("Failed to lock memory (mlockall).");
        }
        memory_locked = true;

        struct sched_param param = {};
        param.sched_priority = sched_get_priority_max(SCHED_FIFO);

        if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
            throw std::runtime_error("Failed to set scheduler.");
        }

        if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
            throw std::runtime_error("Failed to lock memory (mlockall).");
        }

        stack_prefault();

        timespec cycle_time{};
        cycle_time.tv_sec = static_cast<time_t>(period_ / NSEC_PER_SEC);
        cycle_time.tv_nsec = static_cast<long>(period_ % NSEC_PER_SEC);

        timespec wakeup_time{};
        if (clock_gettime(CLOCK_MONOTONIC, &wakeup_time) == -1) {
            throw std::runtime_error("clock_gettime failed.");
        }

        while (running_.load(std::memory_order_acquire)) {
            wakeup_time.tv_nsec += cycle_time.tv_nsec;
            while (wakeup_time.tv_nsec >= NSEC_PER_SEC) {
                wakeup_time.tv_sec++;
                wakeup_time.tv_nsec -= NSEC_PER_SEC;
            }

            int sleep_rc;
            do {
                sleep_rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, nullptr);
            } while (sleep_rc == EINTR);

            if (sleep_rc != 0) {
                throw std::runtime_error("clock_nanosleep failed.");
            }

            rethrowSerialExceptionIfAny();

            for (auto& m_iter : masters_) {
                if (isSerialMaster(m_iter.first)) continue;
                m_iter.second->apply_application_time(wakeup_time);
            }

            for (auto& m_iter : masters_) {
                if (isSerialMaster(m_iter.first)) continue;
                m_iter.second->receive();
            }

            rethrowSerialExceptionIfAny();

            if (is_disabled_.load(std::memory_order_acquire)) {
                break;
            } else if (on_disabled_.load(std::memory_order_acquire)) {
                disableControllers(std::nullopt);
            } else {
                enableControllers(std::nullopt);
                updateControllers(std::nullopt);
            }

            for (auto& m_iter : masters_) {
                if (isSerialMaster(m_iter.first)) continue;
                m_iter.second->save_clock();
            }

            for (auto& m_iter : masters_) {
                if (isSerialMaster(m_iter.first)) continue;
                m_iter.second->transmit();
            }
        }

        rethrowSerialExceptionIfAny();
    } catch (...) {
        if (memory_locked) unlock_memory();
        stopSerialWorkers();
        stop();
        throw;
    }

    if (memory_locked) unlock_memory();
    stopSerialWorkers();
    stop();
}
