#ifndef MASTER_ETHERCAT_MASTER_HPP_
#define MASTER_ETHERCAT_MASTER_HPP_

#include <cstdint>

#include "ecrt.h"

#include "motor_manager/core/motor_master.hpp"

namespace micros {

class EthercatMaster : public MotorMaster {
public:
    explicit EthercatMaster(const master_config_t& config)
    : MotorMaster(config),
      master_idx_(config.master_idx) {}
    ~EthercatMaster() override = default;

    void initialize() override;
    
    void activate() override;
    
    void deactivate() override;

    void check() override;
    
    void transmit() override;
    
    void receive() override;
    
    ec_master_t* master() const { return master_; }
    
    ec_domain_t* domain() const { return domain_; }
    
    uint8_t* domain_pd() const { return domain_pd_; }

    unsigned int master_idx() { return master_idx_; }

private:
    void check_master_state();
    
    void check_domain_state();

    ec_master_t* master_{nullptr};
    
    ec_domain_t* domain_{nullptr};
    
    uint8_t* domain_pd_{nullptr};

    ec_master_state_t master_state_{};
    
    ec_domain_state_t domain_state_{};

    const unsigned int master_idx_;
};

} //namespace micros
#endif // MASTER_ETHERCAT_MASTER_HPP_