#include <cstring>
#include <stdexcept>

#include <linux/can.h>

#include "socketcan/socketcan_controller.hpp"

void socketcan::SocketcanController::initialize(
    motor_interface::MotorMaster& master,
    motor_interface::MotorDriver& driver)
{
    SocketcanMaster* m = dynamic_cast<SocketcanMaster*>(&master);
    if (!m) throw std::runtime_error("Failed to cast master to SocketcanMaster.");
    if (!driver.hasSocketcanFrameCodec()) {
        throw std::runtime_error("Driver does not provide a SocketCAN frame codec.");
    }

    master_ = m;
    driver_ = &driver;

    master_->registerNode(node_id_);

    node_ = master_->node(node_id_);
    if (!node_) throw std::runtime_error("Failed to get SocketCAN node data.");

    status_cache_.controller_index = index_;
    registerEntries();
}

void socketcan::SocketcanController::registerEntries()
{
}

bool socketcan::SocketcanController::enable()
{
    if (current_driver_state_ == motor_interface::DriverState::OperationEnabled) return true;

    motor_interface::socketcan_frame_t frame{};
    if (driver_->encodeSocketcanEnable(node_id_, frame)) {
        enqueueDriverFrame(frame);
    }

    current_driver_state_ = motor_interface::DriverState::OperationEnabled;
    return false;
}

bool socketcan::SocketcanController::disable()
{
    if (current_driver_state_ == motor_interface::DriverState::SwitchOnDisabled) return true;

    motor_interface::socketcan_frame_t frame{};
    if (driver_->encodeSocketcanDisable(node_id_, frame)) {
        enqueueDriverFrame(frame);
    }

    current_driver_state_ = motor_interface::DriverState::SwitchOnDisabled;
    return false;
}

void socketcan::SocketcanController::check(const motor_interface::motor_frame_t& status)
{
    (void)status;
}

void socketcan::SocketcanController::write(const motor_interface::motor_frame_t& command)
{
    if (command.number_of_target_interfaces == 0) return;

    motor_interface::socketcan_frame_t frame{};
    if (!driver_->encodeSocketcanCommand(node_id_, command, frame)) {
        throw std::runtime_error("Failed to encode SocketCAN command frame.");
    }

    enqueueDriverFrame(frame);
}

void socketcan::SocketcanController::read(motor_interface::motor_frame_t& status)
{
    can_frame raw{};
    const bool has_frame = master_->takeReceivedFrame(
        [this](const can_frame& frame) {
            const motor_interface::socketcan_frame_t driver_frame = toDriverFrame(frame);
            return driver_->acceptsSocketcanStatusFrame(node_id_, driver_frame);
        },
        raw);

    if (has_frame) {
        motor_interface::motor_frame_t decoded = status_cache_;
        const motor_interface::socketcan_frame_t driver_frame = toDriverFrame(raw);
        if (driver_->decodeSocketcanStatus(node_id_, driver_frame, decoded)) {
            decoded.controller_index = index_;
            status_cache_ = decoded;
        }
    }

    status = status_cache_;
}

void socketcan::SocketcanController::writeData(
    const motor_interface::entry_table_t* rx_interfaces,
    uint8_t number_of_rx_interfaces)
{
    (void)rx_interfaces;
    (void)number_of_rx_interfaces;
}

void socketcan::SocketcanController::readData(
    motor_interface::entry_table_t* tx_interfaces,
    uint8_t number_of_tx_interfaces)
{
    (void)tx_interfaces;
    (void)number_of_tx_interfaces;
}

void socketcan::SocketcanController::enqueueDriverFrame(
    const motor_interface::socketcan_frame_t& frame)
{
    if (frame.can_dlc > motor_interface::MAX_SOCKETCAN_DATA_SIZE) {
        throw std::runtime_error("Invalid SocketCAN frame size.");
    }

    can_frame raw{};
    raw.can_id = frame.can_id;
    raw.can_dlc = frame.can_dlc;
    std::memcpy(raw.data, frame.data, frame.can_dlc);

    master_->enqueueFrame(raw);
}

motor_interface::socketcan_frame_t socketcan::SocketcanController::toDriverFrame(
    const can_frame& frame) const
{
    motor_interface::socketcan_frame_t driver_frame{};
    driver_frame.can_id = (frame.can_id & CAN_EFF_FLAG) ?
        (frame.can_id & CAN_EFF_MASK) :
        (frame.can_id & CAN_SFF_MASK);
    driver_frame.can_dlc = frame.can_dlc;
    std::memcpy(driver_frame.data, frame.data, frame.can_dlc);

    return driver_frame;
}
