#include <algorithm>
#include <cstring>
#include <stdexcept>

#include "canopen/canopen_controller.hpp"

void canopen::CanopenController::initialize(motor_interface::MotorMaster& master, motor_interface::MotorDriver& driver)
{
    CanopenMaster* m = dynamic_cast<CanopenMaster*>(&master);
    if (!m) throw std::runtime_error("Failed to cast master to CanopenMaster.");

    master_ = m;
    driver_ = &driver;

    master_->registerNodes(node_id_);

    node_ = master_->node(node_id_);
    if (!node_) throw std::runtime_error("Failed to get CANopen node data.");

    registerEntries();
}

void canopen::CanopenController::registerEntries()
{
    addSlaveConfigSdos();
    addSlaveConfigPdos();
}

bool canopen::CanopenController::enable()
{
    uint8_t sw_data[2]{0};
    std::memcpy(sw_data, node_->tpdo + offset_[motor_interface::ID_STATUSWORD], sizeof(sw_data));

    uint8_t cw_data[2]{0};
    if (!driver_->isEnabled(sw_data, current_driver_state_, cw_data)) {
        std::memcpy(node_->rpdo + offset_[motor_interface::ID_CONTROLWORD], cw_data, sizeof(cw_data));

        node_->rpdo_dirty = true;
        return false;
    }

    return true;
}

bool canopen::CanopenController::disable()
{
    uint8_t sw_data[2]{0};
    std::memcpy(sw_data, node_->tpdo + offset_[motor_interface::ID_STATUSWORD], sizeof(sw_data));

    uint8_t cw_data[2]{0};
    if (!driver_->isDisabled(sw_data, current_driver_state_, cw_data)) {
        std::memcpy(node_->rpdo + offset_[motor_interface::ID_CONTROLWORD], cw_data, sizeof(cw_data));

        node_->rpdo_dirty = true;
        return false;
    }

    return true;
}

void canopen::CanopenController::check(const motor_interface::motor_frame_t& status)
{
    uint8_t sw_data[2]{0};
    motor_interface::fill<uint16_t>(status.statusword, sw_data);

    uint8_t cw_data[2]{0};
    if (driver_->isReceived(sw_data, cw_data)) {
        std::memcpy(node_->rpdo + offset_[motor_interface::ID_CONTROLWORD], cw_data, sizeof(cw_data));

        node_->rpdo_dirty = true;
    }
}

void canopen::CanopenController::write(const motor_interface::motor_frame_t& command)
{
    motor_interface::entry_table_t rx_interfaces[motor_interface::MAX_INTERFACE_SIZE]{0};

    const uint8_t n_rx = std::min(command.number_of_target_interfaces, motor_interface::MAX_INTERFACE_SIZE);
    for (uint8_t i = 0; i < n_rx; ++i) {
        const uint8_t id = command.target_interface_id[i];

        if (id == motor_interface::ID_CONTROLWORD) {
            rx_interfaces[i].id = id;
            rx_interfaces[i].type = motor_interface::DataType::U16;
            rx_interfaces[i].size = 2;
            motor_interface::fill<uint16_t>(command.controlword, rx_interfaces[i].data);
        } else if (id == motor_interface::ID_TARGET_POSITION) {
            rx_interfaces[i].id = id;
            rx_interfaces[i].type = motor_interface::DataType::S32;
            rx_interfaces[i].size = 4;
            motor_interface::fill<int32_t>(driver_->position(command.position), rx_interfaces[i].data);
        } else if (id == motor_interface::ID_TARGET_VELOCITY) {
            rx_interfaces[i].id = id;
            rx_interfaces[i].type = motor_interface::DataType::S32;
            rx_interfaces[i].size = 4;
            motor_interface::fill<int32_t>(driver_->velocity(command.velocity), rx_interfaces[i].data);
        } else if (id == motor_interface::ID_TARGET_TORQUE) {
            rx_interfaces[i].id = id;
            rx_interfaces[i].type = motor_interface::DataType::S16;
            rx_interfaces[i].size = 2;
            motor_interface::fill<int16_t>(driver_->torque(command.torque), rx_interfaces[i].data);
        } else {
            throw std::runtime_error("Invalid RX interface ID.");
        }
    }

    writeData(rx_interfaces, n_rx);
}

void canopen::CanopenController::read(motor_interface::motor_frame_t& status)
{
    readData(tx_interfaces_, driver_->number_of_tx_interfaces());

    for (uint8_t i = 0; i < driver_->number_of_tx_interfaces(); ++i) {
        const motor_interface::entry_table_t& e = tx_interfaces_[i];

        if (e.id == motor_interface::ID_STATUSWORD) {
            status.statusword = motor_interface::value<uint16_t>(e.data);
        } else if (e.id == motor_interface::ID_ERRORCODE) {
            status.errorcode = motor_interface::value<uint16_t>(e.data);
        } else if (e.id == motor_interface::ID_CURRENT_POSITION) {
            status.position = driver_->position(motor_interface::value<int32_t>(e.data));
        } else if (e.id == motor_interface::ID_CURRENT_VELOCITY) {
            status.velocity = driver_->velocity(motor_interface::value<int32_t>(e.data));
        } else if (e.id == motor_interface::ID_CURRENT_TORQUE) {
            status.torque = driver_->torque(motor_interface::value<int16_t>(e.data));
        } else {
            throw std::runtime_error("Invalid TX interface ID.");
        }
    }

    status.controller_index = index_;
}

void canopen::CanopenController::writeData(const motor_interface::entry_table_t* rx_interfaces, uint8_t number_of_rx_interfaces)
{
    for (uint8_t i = 0; i < number_of_rx_interfaces; ++i) {
        const motor_interface::entry_table_t& e = rx_interfaces[i];

        std::memcpy(node_->rpdo + offset_[e.id], e.data, e.size);
    }

    node_->rpdo_dirty = true;
}

void canopen::CanopenController::readData(motor_interface::entry_table_t* tx_interfaces, uint8_t number_of_tx_interfaces)
{
    for (uint8_t i = 0; i < number_of_tx_interfaces; ++i) {
        const motor_interface::entry_table_t& e = tx_interfaces[i];

        std::memcpy(node_->tpdo + offset_[e.id], e.data, e.size);
    }

    node_->tpdo_updated = false;
}

void canopen::CanopenController::addSlaveConfigSdos()
{
    const motor_interface::entry_table_t* items = driver_->items();

    for (uint8_t i = 0; i < driver_->number_of_items(); ++i) {
        motor_interface::entry_table_t item = items[i];

        if (item.id == motor_interface::ID_OPERATING_MODE) {
            int8_t value = motor_interface::value<int8_t>(item.data);

            if (profile_mode_ == 0) {
                value = driver_->profile_position_value();
            } else if (profile_mode_ == 1) {
                value = driver_->profile_velocity_value();
            } else if (profile_mode_ == 2) {
                value = driver_->profile_torque_value();
            }

            motor_interface::fill<int8_t>(value, item.data);
        }

        if (!master_->writeSdo(
            node_id_,
            item.index,
            item.subindex,
            item.data,
            item.size
        )) throw std::runtime_error("Failed to write CANopen SDO.");
    }
}

void canopen::CanopenController::addSlaveConfigPdos()
{
    const motor_interface::entry_table_t* interfaces = driver_->interfaces();

    const uint8_t num_rx_interfaces = driver_->number_of_rx_interfaces();
    const uint8_t num_tx_interfaces = driver_->number_of_tx_interfaces();

    uint8_t rpdo_offset = 0;
    uint8_t tpdo_offset = 0;

    for (uint8_t i = 0; i < num_rx_interfaces; ++i) {
        const motor_interface::entry_table_t& e = interfaces[i + 1];

        if (profile_mode_ == 0) {
            if (e.id == motor_interface::ID_TARGET_VELOCITY || e.id == motor_interface::ID_TARGET_TORQUE) {
                continue;
            }
        } else if (profile_mode_ == 1) {
            if (e.id == motor_interface::ID_TARGET_POSITION || e.id == motor_interface::ID_TARGET_TORQUE) {
                continue;
            }
        } else if (profile_mode_ == 2) {
            if (e.id == motor_interface::ID_TARGET_POSITION || e.id == motor_interface::ID_TARGET_VELOCITY) {
                continue;
            }
        }

        if (rpdo_offset + e.size > 16) throw std::runtime_error("RPDO size exceeds 16 bytes.");

        offset_[e.id] = rpdo_offset;
        rpdo_offset += e.size;
    }

    for (uint8_t i = 0; i < num_tx_interfaces; ++i) {
        const motor_interface::entry_table_t& e = interfaces[i + num_rx_interfaces + 2];

        if (tpdo_offset + e.size > 16) throw std::runtime_error("TPDO size exceeds 16 bytes.");

        offset_[e.id] = tpdo_offset;
        tpdo_offset += e.size;

        tx_interfaces_[i] = motor_interface::entry_table_t{
            e.id,
            e.index,
            e.subindex,
            e.type,
            e.size,
            {0}
        };
    }

    rpdo_size_ = rpdo_offset;
    tpdo_size_ = tpdo_offset;

    node_->rpdo_size = rpdo_size_;
    node_->tpdo_size = tpdo_size_;
}