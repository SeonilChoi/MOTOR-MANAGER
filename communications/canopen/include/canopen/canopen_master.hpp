#ifndef CANOPEN_CANOPEN_MASTER_HPP_
#define CANOPEN_CANOPEN_MASTER_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <lely/ev/loop.hpp>
#include <lely/io2/can/msg.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>

#include "motor_interface/motor_master.hpp"

namespace canopen {

class CanopenMaster : public motor_interface::MotorMaster {
public:
    explicit CanopenMaster(const motor_interface::master_config_t& config)
    : motor_interface::MotorMaster(config)
    , can_interface_index_(config.can_interface_index)
    , can_bitrate_(config.can_bitrate) {}

    virtual ~CanopenMaster();

    virtual void initialize() override;

    virtual void activate() override;

    virtual void deactivate() override;

    virtual void transmit() override;

    virtual void receive() override;

    virtual void apply_application_time(const timespec& time) override;

    virtual void save_clock() override;

    void sendFrame(const can_msg& frame);

    void sendNmt(uint8_t command, uint8_t node_id);

    bool latestFrame(uint32_t can_id, can_msg& frame) const;

    void clearFrame(uint32_t can_id);

    bool waitFrame(uint32_t can_id, can_msg& frame, std::chrono::milliseconds timeout);

    unsigned int can_interface_index() const { return can_interface_index_; }

    unsigned int can_bitrate() const { return can_bitrate_; }

private:
    static constexpr std::size_t CAN_SFF_TABLE_SIZE = 0x800;

    std::unique_ptr<lely::io::IoGuard> io_guard_;

    std::unique_ptr<lely::io::Context> context_;

    std::unique_ptr<lely::io::Poll> poll_;

    std::unique_ptr<lely::ev::Loop> loop_;

    std::unique_ptr<lely::io::CanController> can_controller_;

    std::unique_ptr<lely::io::CanChannel> can_channel_;

    std::array<can_msg, CAN_SFF_TABLE_SIZE> latest_frames_{};

    std::array<bool, CAN_SFF_TABLE_SIZE> latest_frame_valid_{};

    bool active_{false};

    std::string interface_name_;

    const unsigned int can_interface_index_{0};

    const unsigned int can_bitrate_{0};
};

} // namespace canopen
#endif // CANOPEN_CANOPEN_MASTER_HPP_
