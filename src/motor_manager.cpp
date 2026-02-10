#include <iostream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

#include "motor_manager/motor_manager.hpp"

#include "motor_manager/master/ethercat_master.hpp"

#include "motor_manager/controller/ethercat_controller.hpp"

#include "motor_manager/driver/minas_driver.hpp"

micros::MotorManager::MotorManager(const std::string& config_file)
{
    load_configurations(config_file);
    initialize_motor_manager();
}

void micros::MotorManager::load_configurations(const std::string& config_file)
{
    YAML::Node root = YAML::LoadFile(config_file);
    if (!root) {
        throw std::runtime_error("Failed to load configuration file.");
    }
    
    period_ = root["period"].as<uint32_t>();

    YAML::Node masters = root["masters"];
    if (!masters || !masters.IsSequence()) {
        throw std::runtime_error("Invalid masters configuration.");
    }

    masters_.reserve(MAX_MASTER_SIZE);

    uint8_t s_idx{0};
    for (const auto& m : masters) {
        master_config_t m_cfg{};
        m_cfg.id = m["id"].as<uint8_t>();
        m_cfg.number_of_slaves = m["number_of_slaves"].as<uint8_t>();
        
        YAML::Node slaves = m["slaves"];
        if (!slaves || !slaves.IsSequence()) {
            throw std::runtime_error("Invalid slaves configuration.");
        }

        switch (to_communication_type(m["type"].as<std::string>())) {
        case CommunicationType::Ethercat: {
            m_cfg.master_idx = m["master_idx"].as<unsigned int>();
            masters_[m_cfg.id] = std::make_unique<EthercatMaster>(m_cfg);
    
            for (uint8_t i = 0; i < m["number_of_slaves"].as<uint8_t>(); ++i) {
                slave_config_t s_cfg{};
                s_cfg.master_id = m_cfg.id;
                s_cfg.driver_id = slaves[i]["driver_id"].as<uint8_t>();
                s_cfg.alias = slaves[i]["alias"].as<uint16_t>();
                s_cfg.position = slaves[i]["position"].as<uint16_t>();
                s_cfg.vid = slaves[i]["vid"].as<uint32_t>();
                s_cfg.pid = slaves[i]["pid"].as<uint32_t>();
                
                controllers_[s_idx] = std::make_unique<EthercatController>(s_cfg);
                s_idx++;
            }
            break;
        }
        default: {
            throw std::runtime_error("Unsupported communication type.");
        }
        }
    }

    YAML::Node drivers = root["drivers"];
    if (!drivers || !drivers.IsSequence()) {
        throw std::runtime_error("Invalid drivers configurations.");
    }

    drivers_.reserve(MAX_CONTROLLER_SIZE);

    for (const auto& d : drivers) {
        driver_config_t d_cfg{};
        d_cfg.id = d["id"].as<uint8_t>();
        d_cfg.pulse_per_revolution = d["pulse_per_revolution"].as<uint32_t>();
        d_cfg.rated_torque = d["rated_torque"].as<double>();
        d_cfg.unit_torque = d["unit_torque"].as<double>();
        d_cfg.lower = d["lower"].as<double>();
        d_cfg.upper = d["upper"].as<double>();
        d_cfg.speed = d["speed"].as<double>();
        d_cfg.profile_velocity = d["profile_velocity"].as<double>();
        d_cfg.profile_acceleration = d["profile_acceleration"].as<double>();
        d_cfg.profile_deceleration = d["profile_deceleration"].as<double>();
        
        switch (to_driver_type(d["type"].as<std::string>())) {
        case DriverType::Minas: {
            drivers_[d_cfg.id] = std::make_unique<MinasDriver>(d_cfg);
            break;
        }
        default: {
            throw std::runtime_error("Unsupported type of driver: " + std::to_string(d_cfg.id));
        }
        }
        drivers_.at(d_cfg.id)->load_parameters(d["param_file"].as<std::string>());
    }

    number_of_slaves_ = s_idx;
}

void micros::MotorManager::initialize_motor_manager()
{    
    frequency_ = NSEC_PER_SEC / period_;

    for (auto & m_iter : masters_) {
        m_iter.second->initialize();
    }
    
    for (uint8_t i = 0; i < number_of_slaves_; ++i) {
        uint8_t mid = controllers_[i]->master_id();
        uint8_t did = controllers_[i]->driver_id();
        controllers_[i]->initialize(*masters_.at(mid), *drivers_.at(did));
    }
}

void micros::MotorManager::start()
{
    for (auto & m_iter : masters_) {
        m_iter.second->activate();
    }
}

void micros::MotorManager::stop()
{
    for (auto & m_iter : masters_) {
        m_iter.second->deactivate();
    }
}

bool micros::MotorManager::update(bool is_interrupt, motor_state_t* states, const motor_state_t* cmds)
{
    for (auto & m_iter : masters_) {
        m_iter.second->receive();
    }

    if (is_interrupt) {
        disable_motor_manager();
    } else {
        if (!is_enable_) {
            enable_motor_manager();
        } else {
            read_motor_state(states);
            check_motor_state(states);
            write_motor_state(cmds);
        }
    }

    for (auto & m_iter : masters_) {
        m_iter.second->transmit();
    }

    return is_disable_;
}

void micros::MotorManager::enable_motor_manager()
{
    uint8_t result[MAX_DRIVER_SIZE] = {0};
    uint8_t sum = 0;
    for (uint8_t i = 0; i < number_of_slaves_; ++i) {
        if (!result[i]) {
            result[i] = controllers_[i]->servo_on();
        }
        sum += result[i];
    }
    if (sum == number_of_slaves_) is_enable_ = true;
}

void micros::MotorManager::disable_motor_manager()
{
    uint8_t result[MAX_DRIVER_SIZE] = {0};
    uint8_t sum = 0;
    for (uint8_t i = 0; i < number_of_slaves_; ++i) {
        if (!result[i]) {
            result[i] = controllers_[i]->servo_off();
        }
        sum += result[i];
    }
    if (sum == number_of_slaves_) is_disable_ = true;
}

void micros::MotorManager::check_motor_state(const motor_state_t* states)
{
    for (uint8_t i = 0; i < number_of_slaves_; ++i) {
        controllers_[i]->check(states[i]);
    }
}

void micros::MotorManager::write_motor_state(const motor_state_t* cmds)
{
    for (uint8_t i = 0; i < number_of_slaves_; ++i) {
        if (cmds[i].number_of_targets > 0) {
            controllers_[cmds[i].id]->write(cmds[i]);
        }
    }
}

void micros::MotorManager::read_motor_state(motor_state_t* states)
{
    for (uint8_t i = 0; i < number_of_slaves_; ++i) {
        controllers_[i]->read(states[i]);
    }
}