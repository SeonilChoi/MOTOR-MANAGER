#include <vector>
#include <cstdio>
#include <stdexcept>

#include "motor_manager/controller/ethercat_controller.hpp"

void micros::EthercatController::initialize(MotorMaster& master, MotorDriver& driver)
{
    auto* m = dynamic_cast<EthercatMaster*>(&master);
    if (!m) {
        throw std::runtime_error("Failed to cast EthercatMaster.");
    }
    master_ = m;
    driver_ = &driver;

    slave_config_ = ecrt_master_slave_config(
        master_->master(), alias_, position_, vid_, pid_
    );
    if (!slave_config_) {
        throw std::runtime_error("Failed to create slave config.");
    }
    
    printf("[EthercatController::initialize][master id: %u][driver id: %u] Initialized controller\n", master_id_, driver_id_);
}

void micros::EthercatController::configure()
{
    add_slave_config_pdos();
    add_slave_config_sdos();
}

bool micros::EthercatController::servo_on()
{
    uint8_t* domain_pd = master_->domain_pd();
    uint16_t sw = EC_READ_U16(domain_pd + offset_[ID_STATUSWORD]);
    
    uint8_t sw_data[2];
    fill_data<uint16_t>(sw, sw_data);
    
    uint8_t cw_data[2];
    if (!(driver_->is_enabled(sw_data, driver_state_, cw_data))) {
        uint16_t cw = to_value<uint16_t>(cw_data);
        EC_WRITE_U16(domain_pd + offset_[ID_CONTROLWORD], cw);
        return false;
    }
    return true;
}

bool micros::EthercatController::servo_off()
{
    uint8_t* domain_pd = master_->domain_pd();
    uint16_t sw = EC_READ_U16(domain_pd + offset_[ID_STATUSWORD]);
    
    uint8_t sw_data[2];
    fill_data<uint16_t>(sw, sw_data);

    uint8_t cw_data[2] = {0};
    if (!(driver_->is_disabled(sw_data, driver_state_, cw_data))) {
        uint16_t cw = to_value<uint16_t>(cw_data);
        EC_WRITE_U16(domain_pd + offset_[ID_CONTROLWORD], cw);
        return false;
    }
    return true;
}

void micros::EthercatController::check(const motor_state_t& state)
{
    uint8_t* domain_pd = master_->domain_pd();
    
    uint8_t sw_data[2];
    fill_data<uint16_t>(state.statusword, sw_data);

    uint8_t cw_data[2]{0};
    if (driver_->is_received(sw_data, cw_data)){
        uint16_t cw = to_value<uint16_t>(cw_data);
        EC_WRITE_U16(domain_pd + offset_[ID_CONTROLWORD], cw);
    }
}

void micros::EthercatController::write(const motor_state_t& cmd)
{
    entry_table_t entries[MAX_INTERFACE_SIZE] = {0};
    for (uint8_t i = 0; i < cmd.number_of_targets; ++i) {
        if (cmd.target_id[i] == ID_CONTROLWORD) {
            entries[i].id = ID_CONTROLWORD;
            entries[i].type = ValueType::U16;
            fill_data<uint16_t>(cmd.controlword, entries[i].data);
        } else if (cmd.target_id[i] == ID_TARGET_POSITION) {
            int32_t value = driver_->position(cmd.position);
            entries[i].id = ID_TARGET_POSITION;
            entries[i].type = ValueType::S32;
            fill_data<int32_t>(value, entries[i].data);
        } else if (cmd.target_id[i] == ID_TARGET_VELOCITY) {
            int32_t value = driver_->velocity(cmd.velocity);
            entries[i].id = ID_TARGET_VELOCITY;
            entries[i].type = ValueType::S32;
            fill_data<int32_t>(value, entries[i].data);
        } else if (cmd.target_id[i] == ID_TARGET_TORQUE) {
            int16_t value = driver_->torque(cmd.torque);
            entries[i].id = ID_TARGET_TORQUE;
            entries[i].type = ValueType::S16;
            fill_data<int16_t>(value, entries[i].data);
        } else {
            throw std::runtime_error("Unsupported interface ID.");
        }
    }
    write_data(entries, cmd.number_of_targets);
}

void micros::EthercatController::read(motor_state_t& state)
{
    read_data(tx_pdos_);
    for (uint8_t i = 0; i < driver_->number_of_tx_pdos(); ++i) {
        if (tx_pdos_[i].id == ID_STATUSWORD) {
            state.statusword = to_value<uint16_t>(tx_pdos_[i].data);
        } else if (tx_pdos_[i].id == ID_ERRORCODE) {
            state.errorcode = to_value<uint16_t>(tx_pdos_[i].data);
        } else if (tx_pdos_[i].id == ID_CURRENT_POSITION) {
            int32_t value = to_value<int32_t>(tx_pdos_[i].data);
            state.position = driver_->position(value);
        } else if (tx_pdos_[i].id == ID_CURRENT_VELOCITY) {
            int32_t value = to_value<int32_t>(tx_pdos_[i].data);
            state.velocity = driver_->velocity(value);
        } else if (tx_pdos_[i].id == ID_CURRENT_TORQUE) {
            int16_t value = to_value<int16_t>(tx_pdos_[i].data);
            state.torque = driver_->torque(value);
        } else {
            throw std::runtime_error("Unsupported interface ID.");
        }
    }
    state.controller_idx = idx_;
}

void micros::EthercatController::write_data(const entry_table_t* pdos, uint8_t size)
{
    uint8_t* domain_pd = master_->domain_pd();
    for (uint8_t i = 0; i < size; ++i) {
        switch (pdos[i].type) {
        case ValueType::U8: {
            uint8_t value = to_value<uint8_t>(pdos[i].data);
            EC_WRITE_U8(domain_pd + offset_[pdos[i].id], value);
            break;
        } case ValueType::U16: {
            uint16_t value = to_value<uint16_t>(pdos[i].data);
            EC_WRITE_U16(domain_pd + offset_[pdos[i].id], value);
            break;
        } case ValueType::U32: {
            uint32_t value = to_value<uint32_t>(pdos[i].data);
            EC_WRITE_U32(domain_pd + offset_[pdos[i].id], value);
            break;
        } case ValueType::S8: {
            int8_t value = to_value<int8_t>(pdos[i].data);
            EC_WRITE_S8(domain_pd + offset_[pdos[i].id], value);
            break;
        } case ValueType::S16: {
            int16_t value = to_value<int16_t>(pdos[i].data);
            EC_WRITE_S16(domain_pd + offset_[pdos[i].id], value);
            break;
        } case ValueType::S32: {
            int32_t value = to_value<int32_t>(pdos[i].data);
            EC_WRITE_S32(domain_pd + offset_[pdos[i].id], value);
            break;
        } default: {
            throw std::runtime_error("Unsupported value type.");
        }
        }
    }
}

void micros::EthercatController::read_data(entry_table_t* pdos)
{
    uint8_t* domain_pd = master_->domain_pd();
    for (uint8_t i = 0; i < driver_->number_of_tx_pdos(); ++i) {
        switch (pdos[i].type) {
        case ValueType::U8: {
            fill_data<uint8_t>(EC_READ_U8(domain_pd + offset_[pdos[i].id]), pdos[i].data);
            break;
        } case ValueType::U16: {
            fill_data<uint16_t>(EC_READ_U16(domain_pd + offset_[pdos[i].id]), pdos[i].data);
            break;
        } case ValueType::U32: {
            fill_data<uint32_t>(EC_READ_U32(domain_pd + offset_[pdos[i].id]), pdos[i].data);
            break;
        } case ValueType::S8: {
            fill_data<int8_t>(EC_READ_S8(domain_pd + offset_[pdos[i].id]), pdos[i].data);
            break;
        } case ValueType::S16: {
            fill_data<int16_t>(EC_READ_S16(domain_pd + offset_[pdos[i].id]), pdos[i].data);
            break;
        } case ValueType::S32: {
            fill_data<int32_t>(EC_READ_S32(domain_pd + offset_[pdos[i].id]), pdos[i].data);
            break;
        } default: {
            throw std::runtime_error("Unsupported value type.");
        }
        }
    }
}

void micros::EthercatController::add_slave_config_sdos()
{
    const entry_table_t* items = driver_->items();
    for (uint8_t i = 0; i < driver_->number_of_items(); ++i) {
        switch(items[i].type) {
        case ValueType::U8: {
            uint8_t value = to_value<uint8_t>(items[i].data);
            ecrt_slave_config_sdo8(slave_config_, items[i].index, items[i].subindex, value);
            break;
        } case ValueType::U16: {
            uint16_t value = to_value<uint16_t>(items[i].data);
            ecrt_slave_config_sdo16(slave_config_, items[i].index, items[i].subindex, value);
            break;
        } case ValueType::U32: {
            uint32_t value = to_value<uint32_t>(items[i].data);
            ecrt_slave_config_sdo32(slave_config_, items[i].index, items[i].subindex, value);
            break;
        } case ValueType::S8: {
            int8_t value = to_value<int8_t>(items[i].data);
            ecrt_slave_config_sdo8(slave_config_, items[i].index, items[i].subindex, value);
            break;
        } case ValueType::S16: {
            int16_t value = to_value<int16_t>(items[i].data);
            ecrt_slave_config_sdo16(slave_config_, items[i].index, items[i].subindex, value);
            break;
        } case ValueType::S32: {
            int32_t value = to_value<int32_t>(items[i].data);
            ecrt_slave_config_sdo32(slave_config_, items[i].index, items[i].subindex, value);
            break;
        } default: {
            throw std::runtime_error("Unsupported value type.");
        }
        }
    }
}

void micros::EthercatController::add_slave_config_pdos()
{
    const entry_table_t* entries = driver_->entries();

    uint8_t num_entries = driver_->number_of_entries();
    uint8_t num_rx_pdos = driver_->number_of_rx_pdos();
    uint8_t num_tx_pdos = driver_->number_of_tx_pdos();
    
    std::vector<ec_pdo_entry_info_t> pdo_entry_infos;
    std::vector<ec_pdo_entry_reg_t> pdo_entry_regs;

    pdo_entry_infos.reserve(num_entries - 2);
    pdo_entry_regs.reserve(num_entries - 1);

    uint16_t rpdo_index = entries[0].index;
    uint16_t tpdo_index = entries[num_rx_pdos + 1].index;

    for (uint8_t i = 0; i < num_rx_pdos; ++i) {
        const auto& e = entries[i + 1];

        pdo_entry_infos.push_back({
            e.index,
            e.subindex,
            static_cast<uint8_t>(e.size * 8)
        });

        unsigned int& offset = offset_[e.id];
        pdo_entry_regs.push_back({
            alias_,
            position_,
            vid_,
            pid_,
            e.index,
            e.subindex,
            &offset,
            0
        });
    }

    for (uint8_t i = 0; i < num_tx_pdos; ++i) {
        const auto& e = entries[i + num_tx_pdos + 2];

        pdo_entry_infos.push_back({
            e.index,
            e.subindex,
            static_cast<uint8_t>(e.size * 8)
        });

        unsigned int& offset = offset_[e.id];
        pdo_entry_regs.push_back({
            alias_,
            position_,
            vid_,
            pid_,
            e.index,
            e.subindex,
            &offset,
            0
        });

        tx_pdos_[i] = entry_table_t{
            e.id,
            e.index,
            e.subindex,
            e.type,
            e.size,
            0
        };
    }

    pdo_entry_regs.push_back(ec_pdo_entry_reg_t{});

    ec_pdo_info_t pdo_infos[] = {
        {rpdo_index, num_rx_pdos, pdo_entry_infos.data()},
        {tpdo_index, num_tx_pdos, pdo_entry_infos.data() + num_rx_pdos}
    };

    ec_sync_info_t sync_infos[] = {
        {0, EC_DIR_OUTPUT, 0, nullptr,       EC_WD_DISABLE},
        {1, EC_DIR_INPUT,  0, nullptr,       EC_WD_DISABLE},
        {2, EC_DIR_OUTPUT, 1, pdo_infos,     EC_WD_DISABLE},
        {3, EC_DIR_INPUT,  1, pdo_infos + 1, EC_WD_DISABLE},
        {0xFF}
    };

    if (ecrt_slave_config_pdos(slave_config_, EC_END, sync_infos)) {
        throw std::runtime_error("Failed to configure PDOs of slave.");
    }

    if (ecrt_domain_reg_pdo_entry_list(master_->domain(), pdo_entry_regs.data())) {
        throw std::runtime_error("Failed to register PDO entries of slave.");
    }
}

void micros::EthercatController::check_slave_config_state()
{
    ec_slave_config_state_t s;
    ecrt_slave_config_state(slave_config_, &s);

    printf("[EthercatController::check_slave_config_state][master id: %u][driver id: %u]", master_id_, driver_id_);
    if (s.al_state != slave_config_state_.al_state) {
        printf(" AL state 0x%02X", s.al_state);
    }
    if (s.online != slave_config_state_.online) {
        printf(" %s", s.online ? "online" : "offline");
    }
    if (s.operational != slave_config_state_.operational) {
        printf(" %soperational", s.operational ? "" : "Not ");
    }
    printf("\n");
    slave_config_state_ = s;
}
