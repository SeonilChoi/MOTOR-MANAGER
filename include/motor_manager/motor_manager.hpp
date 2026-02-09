#ifndef MOTOR_MANAGER_MOTOR_MANAGER_HPP_
#define MOTOR_MANAGER_MOTOR_MANAGER_HPP_

#include <string>
#include <memory>
#include <cstdint>
#include <unordered_map>

#include "motor_manager/core/motor_master.hpp"
#include "motor_manager/core/motor_driver.hpp"
#include "motor_manager/core/motor_controller.hpp"

inline constexpr uint32_t NSEC_PER_SEC = 1e+9;

namespace mmns {

class MotorManager {
public:
    MotorManager(const std::string& config_file);
    ~MotorManager() = default;

    void start();
    
    void stop();
    
    bool update(bool is_interrupt, motor_state_t* states, const motor_state_t* cmds);

    uint32_t period() const { return period_; }
    
    uint8_t number_of_slaves() const { return number_of_slaves_; } 

private:
    void load_configurations(const std::string& config_file);
    
    void initialize_motor_manager();

    void enable_motor_manager();
    
    void disable_motor_manager();

    void check_motor_state(const motor_state_t* states);
    
    void write_motor_state(const motor_state_t* cmds);
    
    void read_motor_state(motor_state_t* states);

    std::unordered_map<uint8_t, std::unique_ptr<MotorMaster>> masters_;
    
    std::unordered_map<uint8_t, std::unique_ptr<MotorDriver>> drivers_;
    
    std::unique_ptr<MotorController> controllers_[MAX_SLAVE_SIZE];

    uint8_t number_of_slaves_{0};
    
    uint32_t period_{0};
    
    uint32_t frequency_{0};

    std::string config_file_;

    bool is_enable_{0};
    
    bool is_disable_{0};
};

} //namespace mmns
#endif
