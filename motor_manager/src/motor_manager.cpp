#include <algorithm>
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

#include "minas/minas_driver.hpp"
#include "zeroerr/zeroerr_driver.hpp"

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

} // namespace

motor_manager::MotorManager::MotorManager(const std::string& config_file)
{
    loadConfigurations(config_file);

    initialize();
}

void motor_manager::MotorManager::loadConfigurations(const std::string& config_file)
{
    YAML::Node root = YAML::LoadFile(config_file);
    if (!root) throw std::runtime_error("Failed to load configuration file.");

    period_ = root["period"].as<uint32_t>();

    YAML::Node masters = root["masters"];
    if (!masters || !masters.IsSequence()) throw std::runtime_error("Invalid masters configuration.");

    uint8_t s_idx{0};
    masters_.reserve(MAX_MASTER_SIZE);
    for (const auto& m : masters) {
        motor_interface::master_config_t m_cfg{};
        m_cfg.id = m["id"].as<uint8_t>();
        m_cfg.number_of_slaves = m["number_of_slaves"].as<uint8_t>();

        YAML::Node slaves = m["slaves"];
        if (!slaves || !slaves.IsSequence()) throw std::runtime_error("Invalid slaves configuration.");

        switch (toCommunicationType(m["type"].as<std::string>())) {
        case CommunicationType::Ethercat: {
            m_cfg.master_index = m["master_index"].as<unsigned int>();
            masters_[m_cfg.id] = std::make_unique<ethercat::EthercatMaster>(m_cfg);

            for (uint8_t i = 0; i < m["number_of_slaves"].as<uint8_t>(); ++i) {
                motor_interface::slave_config_t s_cfg{};
                s_cfg.controller_index = slaves[i]["controller_index"].as<uint8_t>();
                s_cfg.master_id = m_cfg.id;
                s_cfg.driver_id = slaves[i]["driver_id"].as<uint8_t>();
                s_cfg.alias = slaves[i]["alias"].as<uint16_t>();
                s_cfg.position = slaves[i]["position"].as<uint16_t>();
                s_cfg.vendor_id = slaves[i]["vendor_id"].as<uint32_t>();
                s_cfg.product_id = slaves[i]["product_id"].as<uint32_t>();

                controllers_[s_cfg.controller_index] = std::make_unique<ethercat::EthercatController>(s_cfg);
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

    drivers_.reserve(MAX_DRIVER_SIZE);
    for (const auto& d : drivers) {
        motor_interface::driver_config_t d_cfg{};
        d_cfg.id = d["id"].as<uint8_t>();
        d_cfg.pulse_per_revolution = d["pulse_per_revolution"].as<uint32_t>();
        d_cfg.rated_torque = d["rated_torque"].as<double>();
        d_cfg.unit_torque = d["unit_torque"].as<double>();
        d_cfg.lower = d["lower"].as<double>();
        d_cfg.upper = d["upper"].as<double>();
        d_cfg.speed = d["speed"].as<double>();
        d_cfg.acceleration = d["acceleration"].as<double>();
        d_cfg.deceleration = d["deceleration"].as<double>();
        d_cfg.profile_velocity = d["profile_velocity"].as<double>();
        d_cfg.profile_acceleration = d["profile_acceleration"].as<double>();
        d_cfg.profile_deceleration = d["profile_deceleration"].as<double>();

        switch (toDriverType(d["type"].as<std::string>())) {
        case DriverType::Minas: {
            drivers_[d_cfg.id] = std::make_unique<minas::MinasDriver>(d_cfg);
            break;
        } case DriverType::Zeroerr: {
            drivers_[d_cfg.id] = std::make_unique<zeroerr::ZeroerrDriver>(d_cfg);
            break;
        } default: {
            throw std::runtime_error("Invalid driver type.");
        }
        }
        std::string param_path = d["param_file"].as<std::string>();
        const std::filesystem::path param_fs(param_path);
        if (!param_fs.is_absolute()) {
            const std::filesystem::path base =
                std::filesystem::path(config_file).parent_path();
            param_path = (base / param_fs).lexically_normal().string();
        }
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

void motor_manager::MotorManager::enable()
{
    uint8_t result[MAX_CONTROLLER_SIZE]{0};
    uint8_t sum{0};

    for (uint8_t i = 0; i < number_of_controllers_; ++i) {
        if (!result[i]) result[i] = controllers_[i]->enable();
        sum += result[i];
    }
    is_enable_ = sum == number_of_controllers_;
}

void motor_manager::MotorManager::disable()
{
    uint8_t result[MAX_CONTROLLER_SIZE]{0};
    uint8_t sum{0};

    for (uint8_t i = 0; i < number_of_controllers_; ++i) {
        if (!result[i]) result[i] = controllers_[i]->disable();
        sum += result[i];
    }
    is_disabled_ = sum == number_of_controllers_;
}

void motor_manager::MotorManager::write(const motor_interface::motor_frame_t* command, const uint8_t size)
{
    std::lock_guard<std::mutex> lock(frame_mutex_);
    const uint8_t n = std::min(size, MAX_CONTROLLER_SIZE);
    for (uint8_t i = 0; i < n; ++i) {
        command_[i] = command[i];
    }
    is_command_changed_.store(true, std::memory_order_release);
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
}

void motor_manager::MotorManager::update()
{
    std::lock_guard<std::mutex> lock(frame_mutex_);
    for (uint8_t i = 0; i < number_of_controllers_; ++i) {
        controllers_[i]->read(status_[i]);
        controllers_[i]->check(status_[i]);
    }

    for (uint8_t i = 0; i < number_of_controllers_; ++i) {
        if (status_[i].errorcode != 0) return;
    }

    if (is_command_changed_.exchange(false, std::memory_order_acq_rel)) {
        for (uint8_t i = 0; i < number_of_controllers_; ++i) {
            controllers_[i]->write(command_[i]);
        }
    }
}

void motor_manager::MotorManager::run()
{
    running_.store(true, std::memory_order_release);
    start();

    if (sysconf(_SC_PAGESIZE) == -1) {
        stop();
        throw std::runtime_error("sysconf(_SC_PAGESIZE) failed.");
    }

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        stop();
        throw std::runtime_error("Failed to lock memory (mlockall).");
    }

    struct sched_param param = {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);

    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        unlock_memory();
        stop();
        throw std::runtime_error("Failed to set scheduler.");
    }

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        unlock_memory();
        stop();
        throw std::runtime_error("Failed to lock memory (mlockall).");
    }

    stack_prefault();

    timespec cycle_time{};
    cycle_time.tv_sec = static_cast<time_t>(period_ / NSEC_PER_SEC);
    cycle_time.tv_nsec = static_cast<long>(period_ % NSEC_PER_SEC);

    timespec wakeup_time{};
    if (clock_gettime(CLOCK_MONOTONIC, &wakeup_time) == -1) {
        unlock_memory();
        stop();
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
            unlock_memory();
            stop();
            throw std::runtime_error("clock_nanosleep failed.");
        }

        for (auto& m_iter : masters_) m_iter.second->apply_application_time(wakeup_time);

        for (auto& m_iter : masters_) m_iter.second->receive();

        if (is_disabled_) {
            break;
        } else if (on_disabled_.load(std::memory_order_acquire)) {
            disable();
        } else if (!is_enable_) {
            enable();
        } else {
            update();
        }

        for (auto& m_iter : masters_) m_iter.second->save_clock();

        for (auto& m_iter : masters_) m_iter.second->transmit();
    }

    unlock_memory();
    stop();
}
