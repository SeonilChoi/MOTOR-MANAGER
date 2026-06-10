#ifndef CANOPEN_CANOPEN_CONTROLLER_HPP_
#define CANOPEN_CANOPEN_CONTROLLER_HPP_

#include <chrono>
#include <array>
#include <cstdint>
#include <utility>

#include "canopen/canopen_master.hpp"
#include "motor_interface/motor_controller.hpp"

namespace canopen {

class CanopenController : public motor_interface::MotorController {
public:
    explicit CanopenController(const motor_interface::slave_config_t& config)
    : motor_interface::MotorController(config)
    , node_id_(static_cast<uint8_t>(config.position)) {}

    virtual ~CanopenController() = default;

    void initialize(motor_interface::MotorMaster& master, motor_interface::MotorDriver& driver) override;

    void registerEntries() override;

    bool enable() override;

    bool disable() override;

    void check(const motor_interface::motor_frame_t& status) override;

    void write(const motor_interface::motor_frame_t& command) override;

    void read(motor_interface::motor_frame_t& status) override;

private:
    void writeData(const motor_interface::entry_table_t* rx_interfaces, uint8_t number_of_rx_interfaces) override;

    void readData(motor_interface::entry_table_t* tx_interfaces, uint8_t number_of_tx_interfaces) override;

    void configureItems();

    void configurePdos();

    void configureRemotePdoMapping(
        uint16_t communication_index,
        uint16_t mapping_index,
        uint16_t assignment_index,
        uint32_t cob_id,
        const motor_interface::entry_table_t* entries,
        uint8_t count);

    const motor_interface::entry_table_t& rxInterfaceById(uint8_t id) const;

    const motor_interface::entry_table_t& txInterfaceById(uint8_t id) const;

    template <typename T>
    T readTpdo(const motor_interface::entry_table_t& entry);

    template <typename T>
    void writeRpdo(const motor_interface::entry_table_t& entry, T value);

    template <typename T>
    void writeSdo(uint16_t index, uint8_t subindex, T value);

    CanopenMaster* master_{nullptr};

    motor_interface::entry_table_t rx_interfaces_[motor_interface::MAX_INTERFACE_SIZE]{};

    motor_interface::entry_table_t tx_interfaces_[motor_interface::MAX_INTERFACE_SIZE]{};

    std::array<uint8_t, motor_interface::MAX_INTERFACE_SIZE> rx_offsets_{};

    std::array<uint8_t, motor_interface::MAX_INTERFACE_SIZE> tx_offsets_{};

    uint8_t number_of_rx_interfaces_{0};

    uint8_t number_of_tx_interfaces_{0};

    can_msg rpdo_frame_ = CAN_MSG_INIT;

    uint32_t rpdo_cob_id_{0};

    uint32_t tpdo_cob_id_{0};

    const uint8_t node_id_;

    bool items_configured_{false};

    bool pdos_configured_{false};

    bool operational_requested_{false};

    static constexpr auto SDO_TIMEOUT = std::chrono::milliseconds(1000);
};

} // namespace canopen
#endif // CANOPEN_CANOPEN_CONTROLLER_HPP_
