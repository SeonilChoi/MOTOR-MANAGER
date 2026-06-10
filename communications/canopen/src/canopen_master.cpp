#include <cerrno>
#include <chrono>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>

#include "canopen/canopen_master.hpp"

canopen::CanopenMaster::~CanopenMaster() = default;

void canopen::CanopenMaster::initialize()
{
    interface_name_ = "can" + std::to_string(can_interface_index_);

    io_guard_ = std::make_unique<lely::io::IoGuard>();
    context_ = std::make_unique<lely::io::Context>();
    poll_ = std::make_unique<lely::io::Poll>(*context_);
    loop_ = std::make_unique<lely::ev::Loop>(poll_->get_poll());
    auto executor = loop_->get_executor();

    can_controller_ = std::make_unique<lely::io::CanController>(interface_name_.c_str());
    if (can_bitrate_ > 0) {
        int current_bitrate = 0;
        can_controller_->get_bitrate(&current_bitrate);
        if (current_bitrate != static_cast<int>(can_bitrate_)) {
            can_controller_->set_bitrate(static_cast<int>(can_bitrate_));
            can_controller_->restart();
        } else if (can_controller_->stopped()) {
            can_controller_->restart();
        }
    }
    can_channel_ = std::make_unique<lely::io::CanChannel>(*poll_, executor);
    can_channel_->open(*can_controller_);
}

void canopen::CanopenMaster::activate()
{
    if (!can_controller_ || !can_channel_) throw std::runtime_error("CAN channel is not initialized.");
    if (!can_channel_->is_open()) {
        can_channel_->open(*can_controller_);
    }
    latest_frame_valid_.fill(false);
    active_ = true;
}

void canopen::CanopenMaster::deactivate()
{
    active_ = false;
    if (can_channel_ && can_channel_->is_open()) {
        can_channel_->close();
    }
}

void canopen::CanopenMaster::transmit()
{
}

void canopen::CanopenMaster::receive()
{
    if (!active_ || !can_channel_ || !can_channel_->is_open()) return;

    while (true) {
        can_msg frame = CAN_MSG_INIT;
        std::error_code ec;
        const int result = can_channel_->read(&frame, nullptr, nullptr, 0, ec);
        if (ec) {
            if (ec.value() == EAGAIN || ec.value() == EWOULDBLOCK) break;
            throw std::system_error(ec, "CANopen event loop read");
        }
        if (result <= 0) break;

        if ((frame.flags & CAN_FLAG_IDE) == 0) {
            const uint32_t can_id = frame.id & CAN_MASK_BID;
            if (can_id < CAN_SFF_TABLE_SIZE) {
                latest_frames_[can_id] = frame;
                latest_frame_valid_[can_id] = true;
            }
        }
    }
}

void canopen::CanopenMaster::apply_application_time(const timespec& time)
{
    (void)time;
}

void canopen::CanopenMaster::save_clock()
{
}

void canopen::CanopenMaster::sendFrame(const can_msg& frame)
{
    if (!can_channel_ || !can_channel_->is_open()) throw std::runtime_error("CAN channel is not initialized.");
    if (!active_) throw std::runtime_error("CAN channel is not active.");
    can_channel_->write(frame, -1);
}

void canopen::CanopenMaster::sendNmt(uint8_t command, uint8_t node_id)
{
    can_msg frame = CAN_MSG_INIT;
    frame.id = 0x000;
    frame.len = 2;
    frame.data[0] = command;
    frame.data[1] = node_id;
    sendFrame(frame);
}

bool canopen::CanopenMaster::latestFrame(uint32_t can_id, can_msg& frame) const
{
    can_id &= CAN_MASK_BID;
    if (can_id >= CAN_SFF_TABLE_SIZE || !latest_frame_valid_[can_id]) return false;

    frame = latest_frames_[can_id];
    return true;
}

void canopen::CanopenMaster::clearFrame(uint32_t can_id)
{
    can_id &= CAN_MASK_BID;
    if (can_id < CAN_SFF_TABLE_SIZE) latest_frame_valid_[can_id] = false;
}

bool canopen::CanopenMaster::waitFrame(
    uint32_t can_id,
    can_msg& frame,
    std::chrono::milliseconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    clearFrame(can_id);

    while (std::chrono::steady_clock::now() < deadline) {
        receive();
        if (latestFrame(can_id, frame)) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}
