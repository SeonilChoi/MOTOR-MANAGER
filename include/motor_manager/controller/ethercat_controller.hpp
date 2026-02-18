#ifndef CONTROLLER_ETHERCAT_CONTROLLER_HPP_
#define CONTROLLER_ETHERCAT_CONTROLLER_HPP_

#include <cstdint>

#include "motor_manager/core/motor_controller.hpp"
#include "motor_manager/master/ethercat_master.hpp"

namespace micros {

class EthercatController : public MotorController {
public:
    EthercatController(const slave_config_t& config)
    : MotorController(config),
      alias_(config.alias),
      position_(config.position),
      vid_(config.vid),
      pid_(config.pid) {}
    ~EthercatController() override = default;

    void initialize(MotorMaster& master, MotorDriver& driver) override;
    
    void configure() override;

    bool servo_on() override;
    
    bool servo_off() override;

    void check(const motor_state_t& state) override;
    
    void write(const motor_state_t& cmd) override;
    
    void read(motor_state_t& state) override;

private:
    void write_data(const entry_table_t* pdos, uint8_t size) override;
    
    void read_data(entry_table_t* pdos) override;

    void add_slave_config_sdos();
    
    void add_slave_config_pdos();

    void check_slave_config_state();

    EthercatMaster* master_{nullptr};

    ec_slave_config_t* slave_config_{nullptr};
    
    ec_slave_config_state_t slave_config_state_{};
    
    entry_table_t tx_pdos_[MAX_INTERFACE_SIZE]{};

    unsigned int offset_[MAX_INTERFACE_SIZE]{};
    
    const uint16_t alias_;
    
    const uint16_t position_;
    
    const uint32_t vid_;
    
    const uint32_t pid_; 
};

} //namespace micros
#endif // CONTROLLER_ETHERCAT_CONTROLLER_HPP_