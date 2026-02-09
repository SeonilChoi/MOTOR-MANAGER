#include <cstdio>
#include <stdexcept>

#include "motor_manager/master/ethercat_master.hpp"

void mmns::EthercatMaster::initialize()
{
    master_ = ecrt_request_master(master_idx_);
    if (master_ == nullptr) {
        throw std::runtime_error("Failed to request master.");
    }
    
    domain_ = ecrt_master_create_domain(master_);
    if (domain_ == nullptr) {
        throw std::runtime_error("Failed to create domain of master.");
    }

    printf("[EthercatMaster::initialize][master id: %u] Succeed initialization.\n", id_);
}

void mmns::EthercatMaster::activate()
{
    if (ecrt_master_activate(master_)) {
        throw std::runtime_error("Failed to activate master.");
    }

    if (!(domain_pd_ = ecrt_domain_data(domain_))) {
        throw std::runtime_error("Failed to get domain data.");
    }

    printf("[EthercatMaster::activate][master id: %u] Active.\n", id_);
}

void mmns::EthercatMaster::deactivate()
{
    if (ecrt_master_deactivate(master_)) {
        throw std::runtime_error("Failed to deactivate master.");
    }

    printf("[EthercatMaster::deactivate][master id: %u] Deactive.\n", id_);
}

void mmns::EthercatMaster::check()
{
    check_master_state();
    check_domain_state();
}

void mmns::EthercatMaster::transmit()
{
    if (ecrt_domain_queue(domain_)) {
        throw std::runtime_error("Failed to queue datagrams.");
    }
    if (ecrt_master_send(master_)) {
        throw std::runtime_error("Failed to send datagrams.");
    }
}

void mmns::EthercatMaster::receive()
{
    if (ecrt_master_receive(master_)) {
        throw std::runtime_error("Failed to receive frames.");
    }
    if (ecrt_domain_process(domain_)) {
        throw std::runtime_error("Failed to determine the states of the domain.");
    }
}

void mmns::EthercatMaster::check_master_state()
{
    ec_master_state_t ms;
    ecrt_master_state(master_, &ms);

    printf("[EthercatMaster::check_master_state][master id: %u]", id_);
    if (ms.slaves_responding != master_state_.slaves_responding) {
        printf(" Number of slaves: %u", ms.slaves_responding);       
    }
    if (ms.al_states != master_state_.al_states) {
        printf(" AL state: 0x%02X", ms.al_states);
    }
    if (ms.link_up != master_state_.link_up) {
        printf(" Link is %s", ms.link_up ? "up" : "down");
    }
    printf("\n");
    master_state_ = ms;
}

void mmns::EthercatMaster::check_domain_state()
{
    ec_domain_state_t ds;
    ecrt_domain_state(domain_, &ds);

    printf("[EthercatMaster::check_domain_state][master id: %u]", id_);
    if (ds.working_counter != domain_state_.working_counter) {
        printf(" Working counter: %u", ds.working_counter);
    }
    if (ds.wc_state != domain_state_.wc_state) {
        printf(" Domain state: %u", ds.wc_state);
    }
    printf("\n");
    domain_state_ = ds;
}