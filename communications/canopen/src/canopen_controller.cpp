#include <algorithm>
#include <chrono>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include "canopen/canopen_controller.hpp"

namespace {

constexpr uint8_t NMT_START = 0x01;
constexpr uint8_t NMT_ENTER_PREOP = 0x80;

uint32_t pdo_mapping_value(const motor_interface::entry_table_t& entry)
{
    return (static_cast<uint32_t>(entry.index) << 16)
        | (static_cast<uint32_t>(entry.subindex) << 8)
        | (static_cast<uint32_t>(entry.size) * 8);
}

template <typename T>
T read_entry_value(const motor_interface::entry_table_t& entry)
{
    return motor_interface::value<T>(entry.data);
}

template <typename T>
void write_le(T value, uint8_t* data)
{
    motor_interface::fill<T>(value, data);
}

bool is_profile_rx_interface(uint8_t id, uint8_t profile_mode)
{
    if (id == motor_interface::ID_TARGET_POSITION) return profile_mode == 0;
    if (id == motor_interface::ID_TARGET_VELOCITY) return profile_mode == 1;
    if (id == motor_interface::ID_TARGET_TORQUE) return profile_mode == 2;
    return true;
}

bool is_profile_tx_interface(uint8_t id, uint8_t profile_mode)
{
    if (id == motor_interface::ID_CURRENT_POSITION) return profile_mode == 0;
    if (id == motor_interface::ID_CURRENT_VELOCITY) return profile_mode == 1;
    if (id == motor_interface::ID_CURRENT_TORQUE) return profile_mode == 2;
    return true;
}

} // namespace

void canopen::CanopenController::initialize(
    motor_interface::MotorMaster& master,
    motor_interface::MotorDriver& driver)
{
    CanopenMaster* m = dynamic_cast<CanopenMaster*>(&master);
    if (!m) throw std::runtime_error("Failed to cast master to CanopenMaster.");
    if (node_id_ == 0 || node_id_ > 127) throw std::runtime_error("Invalid CANopen node ID.");
    if (profile_mode_ > 2) throw std::runtime_error("Invalid CANopen profile mode.");

    master_ = m;
    driver_ = &driver;

    registerEntries();
}

void canopen::CanopenController::registerEntries()
{
    const motor_interface::entry_table_t* interfaces = driver_->interfaces();
    const uint8_t configured_rx = driver_->number_of_rx_interfaces();
    const uint8_t configured_tx = driver_->number_of_tx_interfaces();

    uint8_t rx_offset = 0;
    number_of_rx_interfaces_ = 0;
    for (uint8_t i = 0; i < configured_rx; ++i) {
        const auto& e = interfaces[i + 1];
        if (!is_profile_rx_interface(e.id, profile_mode_)) continue;
        if (e.id >= motor_interface::MAX_INTERFACE_SIZE) {
            throw std::runtime_error("CANopen RX interface ID is out of range.");
        }
        if (number_of_rx_interfaces_ >= motor_interface::MAX_INTERFACE_SIZE) {
            throw std::runtime_error("Too many CANopen RX interfaces.");
        }
        if (rx_offset + e.size > CAN_MAX_LEN) {
            throw std::runtime_error("CANopen RPDO payload is too large.");
        }
        rx_offsets_[e.id] = rx_offset;
        rx_offset += e.size;
        rx_interfaces_[number_of_rx_interfaces_++] = e;
    }

    uint8_t tx_offset = 0;
    number_of_tx_interfaces_ = 0;
    for (uint8_t i = 0; i < configured_tx; ++i) {
        const motor_interface::entry_table_t& e = interfaces[i + configured_rx + 2];
        if (!is_profile_tx_interface(e.id, profile_mode_)) continue;
        if (e.id >= motor_interface::MAX_INTERFACE_SIZE) {
            throw std::runtime_error("CANopen TX interface ID is out of range.");
        }
        if (number_of_tx_interfaces_ >= motor_interface::MAX_INTERFACE_SIZE) {
            throw std::runtime_error("Too many CANopen TX interfaces.");
        }
        if (tx_offset + e.size > CAN_MAX_LEN) {
            throw std::runtime_error("CANopen TPDO payload is too large.");
        }
        tx_offsets_[e.id] = tx_offset;
        tx_offset += e.size;
        tx_interfaces_[number_of_tx_interfaces_++] = motor_interface::entry_table_t{
            e.id,
            e.index,
            e.subindex,
            e.type,
            e.size,
            0
        };
    }

    rpdo_frame_ = CAN_MSG_INIT;
    rpdo_frame_.id = rpdo_cob_id_;
    rpdo_frame_.len = rx_offset;
}

bool canopen::CanopenController::enable()
{
    if (!pdos_configured_) configurePdos();
    if (!items_configured_) configureItems();
    if (!operational_requested_) {
        master_->sendNmt(NMT_START, node_id_);
        operational_requested_ = true;
        return false;
    }

    const auto& sw_entry = txInterfaceById(motor_interface::ID_STATUSWORD);
    const uint16_t sw = readTpdo<uint16_t>(sw_entry);

    uint8_t sw_data[2];
    motor_interface::fill<uint16_t>(sw, sw_data);

    uint8_t cw_data[2]{0};
    if (!(driver_->isEnabled(sw_data, current_driver_state_, cw_data))) {
        const auto& cw_entry = rxInterfaceById(motor_interface::ID_CONTROLWORD);
        writeRpdo<uint16_t>(cw_entry, motor_interface::value<uint16_t>(cw_data));
        master_->sendFrame(rpdo_frame_);
        return false;
    }
    return true;
}

bool canopen::CanopenController::disable()
{
    const auto& sw_entry = txInterfaceById(motor_interface::ID_STATUSWORD);
    const uint16_t sw = readTpdo<uint16_t>(sw_entry);

    uint8_t sw_data[2];
    motor_interface::fill<uint16_t>(sw, sw_data);

    uint8_t cw_data[2]{0};
    if (!(driver_->isDisabled(sw_data, current_driver_state_, cw_data))) {
        const auto& cw_entry = rxInterfaceById(motor_interface::ID_CONTROLWORD);
        writeRpdo<uint16_t>(cw_entry, motor_interface::value<uint16_t>(cw_data));
        master_->sendFrame(rpdo_frame_);
        return false;
    }
    return true;
}

void canopen::CanopenController::check(const motor_interface::motor_frame_t& status)
{
    uint8_t sw_data[2];
    motor_interface::fill<uint16_t>(status.statusword, sw_data);

    uint8_t cw_data[2]{0};
    if (driver_->isReceived(sw_data, cw_data)) {
        const auto& cw_entry = rxInterfaceById(motor_interface::ID_CONTROLWORD);
        writeRpdo<uint16_t>(cw_entry, motor_interface::value<uint16_t>(cw_data));
        master_->sendFrame(rpdo_frame_);
    }
}

void canopen::CanopenController::write(const motor_interface::motor_frame_t& command)
{
    motor_interface::entry_table_t rx_interfaces[motor_interface::MAX_INTERFACE_SIZE]{0};
    const uint8_t n_rx = std::min(command.number_of_target_interfaces, motor_interface::MAX_INTERFACE_SIZE);

    for (uint8_t i = 0; i < n_rx; ++i) {
        const uint8_t id = command.target_interface_id[i];
        rx_interfaces[i] = rxInterfaceById(id);

        if (id == motor_interface::ID_CONTROLWORD) {
            motor_interface::fill<uint16_t>(command.controlword, rx_interfaces[i].data);
        } else if (id == motor_interface::ID_TARGET_POSITION) {
            motor_interface::fill<int32_t>(driver_->position(command.position), rx_interfaces[i].data);
        } else if (id == motor_interface::ID_TARGET_VELOCITY) {
            motor_interface::fill<int32_t>(driver_->velocity(command.velocity), rx_interfaces[i].data);
        } else if (id == motor_interface::ID_TARGET_TORQUE) {
            motor_interface::fill<int16_t>(driver_->torque(command.torque), rx_interfaces[i].data);
        } else {
            throw std::runtime_error("Invalid RX interface ID.");
        }
    }
    writeData(rx_interfaces, n_rx);
}

void canopen::CanopenController::read(motor_interface::motor_frame_t& status)
{
    readData(tx_interfaces_, number_of_tx_interfaces_);
    for (uint8_t i = 0; i < number_of_tx_interfaces_; ++i) {
        if (tx_interfaces_[i].id == motor_interface::ID_STATUSWORD) {
            status.statusword = read_entry_value<uint16_t>(tx_interfaces_[i]);
        } else if (tx_interfaces_[i].id == motor_interface::ID_ERRORCODE) {
            status.errorcode = read_entry_value<uint16_t>(tx_interfaces_[i]);
        } else if (tx_interfaces_[i].id == motor_interface::ID_CURRENT_POSITION) {
            status.position = driver_->position(read_entry_value<int32_t>(tx_interfaces_[i]));
        } else if (tx_interfaces_[i].id == motor_interface::ID_CURRENT_VELOCITY) {
            status.velocity = driver_->velocity(read_entry_value<int32_t>(tx_interfaces_[i]));
        } else if (tx_interfaces_[i].id == motor_interface::ID_CURRENT_TORQUE) {
            status.torque = driver_->torque(read_entry_value<int16_t>(tx_interfaces_[i]));
        } else {
            throw std::runtime_error("Invalid TX interface ID.");
        }
    }
    status.controller_index = index_;
}

void canopen::CanopenController::writeData(
    const motor_interface::entry_table_t* rx_interfaces,
    uint8_t number_of_rx_interfaces)
{
    for (uint8_t i = 0; i < number_of_rx_interfaces; ++i) {
        const auto& e = rx_interfaces[i];
        switch (e.type) {
        case motor_interface::DataType::U8:
            writeRpdo<uint8_t>(e, read_entry_value<uint8_t>(e));
            break;
        case motor_interface::DataType::U16:
            writeRpdo<uint16_t>(e, read_entry_value<uint16_t>(e));
            break;
        case motor_interface::DataType::U32:
            writeRpdo<uint32_t>(e, read_entry_value<uint32_t>(e));
            break;
        case motor_interface::DataType::S8:
            writeRpdo<int8_t>(e, read_entry_value<int8_t>(e));
            break;
        case motor_interface::DataType::S16:
            writeRpdo<int16_t>(e, read_entry_value<int16_t>(e));
            break;
        case motor_interface::DataType::S32:
            writeRpdo<int32_t>(e, read_entry_value<int32_t>(e));
            break;
        default:
            throw std::runtime_error("Invalid interface data type.");
        }
    }
    master_->sendFrame(rpdo_frame_);
}

void canopen::CanopenController::readData(
    motor_interface::entry_table_t* tx_interfaces,
    uint8_t number_of_tx_interfaces)
{
    for (uint8_t i = 0; i < number_of_tx_interfaces; ++i) {
        auto& e = tx_interfaces[i];
        switch (e.type) {
        case motor_interface::DataType::U8:
            motor_interface::fill<uint8_t>(readTpdo<uint8_t>(e), e.data);
            break;
        case motor_interface::DataType::U16:
            motor_interface::fill<uint16_t>(readTpdo<uint16_t>(e), e.data);
            break;
        case motor_interface::DataType::U32:
            motor_interface::fill<uint32_t>(readTpdo<uint32_t>(e), e.data);
            break;
        case motor_interface::DataType::S8:
            motor_interface::fill<int8_t>(readTpdo<int8_t>(e), e.data);
            break;
        case motor_interface::DataType::S16:
            motor_interface::fill<int16_t>(readTpdo<int16_t>(e), e.data);
            break;
        case motor_interface::DataType::S32:
            motor_interface::fill<int32_t>(readTpdo<int32_t>(e), e.data);
            break;
        default:
            throw std::runtime_error("Invalid interface data type.");
        }
    }
}

void canopen::CanopenController::configureItems()
{
    const motor_interface::entry_table_t* items = driver_->items();
    for (uint8_t i = 0; i < driver_->number_of_items(); ++i) {
        motor_interface::entry_table_t item = items[i];
        if (item.index == 0x6060 && item.subindex == 0x00) {
            int8_t value = read_entry_value<int8_t>(item);
            if (profile_mode_ == 0) {
                value = driver_->profile_position_value();
            } else if (profile_mode_ == 1) {
                value = driver_->profile_velocity_value();
            } else if (profile_mode_ == 2) {
                value = driver_->profile_torque_value();
            }
            motor_interface::fill<int8_t>(value, item.data);
        }

        switch (item.type) {
        case motor_interface::DataType::U8:
            writeSdo<uint8_t>(item.index, item.subindex, read_entry_value<uint8_t>(item));
            break;
        case motor_interface::DataType::U16:
            writeSdo<uint16_t>(item.index, item.subindex, read_entry_value<uint16_t>(item));
            break;
        case motor_interface::DataType::U32:
            writeSdo<uint32_t>(item.index, item.subindex, read_entry_value<uint32_t>(item));
            break;
        case motor_interface::DataType::S8:
            writeSdo<int8_t>(item.index, item.subindex, read_entry_value<int8_t>(item));
            break;
        case motor_interface::DataType::S16:
            writeSdo<int16_t>(item.index, item.subindex, read_entry_value<int16_t>(item));
            break;
        case motor_interface::DataType::S32:
            writeSdo<int32_t>(item.index, item.subindex, read_entry_value<int32_t>(item));
            break;
        default:
            throw std::runtime_error("Invalid item data type.");
        }
    }
    items_configured_ = true;
}

void canopen::CanopenController::configurePdos()
{
    const motor_interface::entry_table_t* interfaces = driver_->interfaces();
    const uint8_t configured_rx = driver_->number_of_rx_interfaces();
    const uint8_t configured_tx = driver_->number_of_tx_interfaces();

    if (driver_->number_of_interfaces() < static_cast<uint8_t>(configured_rx + configured_tx + 2)) {
        throw std::runtime_error("Invalid CANopen PDO interface configuration.");
    }

    const uint16_t rpdo_index = interfaces[0].index;
    const uint16_t tpdo_index = interfaces[configured_rx + 1].index;

    if (rpdo_index < 0x1600 || rpdo_index > 0x1603 || tpdo_index < 0x1A00 || tpdo_index > 0x1A03) {
        throw std::runtime_error("Unsupported CANopen PDO mapping index.");
    }
    if (number_of_rx_interfaces_ == 0 || number_of_tx_interfaces_ == 0) {
        throw std::runtime_error("CANopen PDO interface list is empty.");
    }

    const uint8_t rpdo_number = static_cast<uint8_t>(rpdo_index - 0x1600 + 1);
    const uint8_t tpdo_number = static_cast<uint8_t>(tpdo_index - 0x1A00 + 1);

    rpdo_cob_id_ = 0x200 + static_cast<uint32_t>(rpdo_number - 1) * 0x100 + node_id_;
    tpdo_cob_id_ = 0x180 + static_cast<uint32_t>(tpdo_number - 1) * 0x100 + node_id_;
    rpdo_frame_.id = rpdo_cob_id_;

    master_->sendNmt(NMT_ENTER_PREOP, node_id_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    master_->receive();

    configureRemotePdoMapping(
        0x1400 + rpdo_number - 1,
        rpdo_index,
        0x1C12,
        rpdo_cob_id_,
        rx_interfaces_,
        number_of_rx_interfaces_);
    configureRemotePdoMapping(
        0x1800 + tpdo_number - 1,
        tpdo_index,
        0x1C13,
        tpdo_cob_id_,
        tx_interfaces_,
        number_of_tx_interfaces_);

    pdos_configured_ = true;
}

void canopen::CanopenController::configureRemotePdoMapping(
    uint16_t communication_index,
    uint16_t mapping_index,
    uint16_t assignment_index,
    uint32_t cob_id,
    const motor_interface::entry_table_t* entries,
    uint8_t count)
{
    constexpr uint32_t PDO_DISABLED = 0x80000000;

    writeSdo<uint32_t>(communication_index, 0x01, cob_id | PDO_DISABLED);
    writeSdo<uint8_t>(mapping_index, 0x00, 0);
    for (uint8_t i = 0; i < count; ++i) {
        writeSdo<uint32_t>(mapping_index, static_cast<uint8_t>(i + 1), pdo_mapping_value(entries[i]));
    }
    writeSdo<uint8_t>(mapping_index, 0x00, count);
    writeSdo<uint8_t>(assignment_index, 0x00, 0);
    writeSdo<uint16_t>(assignment_index, 0x01, mapping_index);
    writeSdo<uint8_t>(assignment_index, 0x00, 1);
    writeSdo<uint32_t>(communication_index, 0x01, cob_id);
}

const motor_interface::entry_table_t& canopen::CanopenController::rxInterfaceById(uint8_t id) const
{
    for (uint8_t i = 0; i < number_of_rx_interfaces_; ++i) {
        if (rx_interfaces_[i].id == id) return rx_interfaces_[i];
    }
    throw std::runtime_error("CANopen RX interface ID is not configured.");
}

const motor_interface::entry_table_t& canopen::CanopenController::txInterfaceById(uint8_t id) const
{
    for (uint8_t i = 0; i < number_of_tx_interfaces_; ++i) {
        if (tx_interfaces_[i].id == id) return tx_interfaces_[i];
    }
    throw std::runtime_error("CANopen TX interface ID is not configured.");
}

template <typename T>
T canopen::CanopenController::readTpdo(const motor_interface::entry_table_t& entry)
{
    can_msg frame = CAN_MSG_INIT;
    if (!master_->latestFrame(tpdo_cob_id_, frame)) return T{};

    const uint8_t offset = tx_offsets_[entry.id];
    if (offset + sizeof(T) > frame.len) return T{};

    return motor_interface::value<T>(frame.data + offset);
}

template <typename T>
void canopen::CanopenController::writeRpdo(const motor_interface::entry_table_t& entry, T value)
{
    const uint8_t offset = rx_offsets_[entry.id];
    if (offset + sizeof(T) > CAN_MAX_LEN) throw std::runtime_error("RPDO payload is too large.");

    write_le<T>(value, rpdo_frame_.data + offset);
}

template <typename T>
void canopen::CanopenController::writeSdo(uint16_t index, uint8_t subindex, T value)
{
    static_assert(sizeof(T) <= 4, "Only expedited SDO writes are supported.");

    can_msg request = CAN_MSG_INIT;
    request.id = 0x600 + node_id_;
    request.len = 8;
    request.data[0] = static_cast<uint8_t>(0x23 | ((4 - sizeof(T)) << 2));
    request.data[1] = static_cast<uint8_t>(index & 0xFF);
    request.data[2] = static_cast<uint8_t>((index >> 8) & 0xFF);
    request.data[3] = subindex;
    write_le<T>(std::move(value), request.data + 4);

    can_msg response = CAN_MSG_INIT;
    master_->sendFrame(request);
    if (!master_->waitFrame(0x580 + node_id_, response, SDO_TIMEOUT)) {
        std::ostringstream oss;
        oss << "CANopen SDO download timeout: node=" << static_cast<int>(node_id_)
            << " index=0x" << std::hex << index
            << " subindex=0x" << static_cast<int>(subindex);
        throw std::runtime_error(oss.str());
    }

    if (response.len < 4 || response.data[0] == 0x80) {
        std::ostringstream oss;
        oss << "CANopen SDO download aborted: node=" << static_cast<int>(node_id_)
            << " index=0x" << std::hex << index
            << " subindex=0x" << static_cast<int>(subindex);
        throw std::runtime_error(oss.str());
    }
    if (response.data[0] != 0x60 || response.data[1] != request.data[1]
        || response.data[2] != request.data[2] || response.data[3] != subindex) {
        throw std::runtime_error("Unexpected CANopen SDO download response.");
    }
}
