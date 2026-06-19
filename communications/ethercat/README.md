# ethercat

IgH EtherCAT transport for `motor_manager`.

This module provides:

- `ethercat::EthercatMaster`, derived from `motor_interface::MotorMaster`
- `ethercat::EthercatController`, derived from `motor_interface::MotorController`

It maps driver SDO/PDO tables to IgH EtherCAT slave configuration and the domain process image.

## Requirements

The top-level `motor_manager` package requires:

- `ecrt.h`
- `libethercat`

The root `CMakeLists.txt` searches `/usr/local`, `/usr`, and `/opt/etherlab`. Set `MOTOR_MANAGER_IGH_ETHERCAT_INCLUDE_DIR` and `MOTOR_MANAGER_IGH_ETHERCAT_LIB` when the IgH install is elsewhere.

## Device setup

```bash
sudo ethercatctl restart
sudo chmod 666 /dev/EtherCAT0
```

## YAML fields

EtherCAT masters are selected with `masters[].type: ethercat`.

```yaml
masters:
  - id: 0
    type: ethercat
    number_of_slaves: 1
    ethercat_master_index: 0
    slaves:
      - controller_index: 0
        driver_id: 0
        alias: 0
        position: 0
        vendor_id: 0x5a65726f
        product_id: 0x00029252
        profile_mode: 0
```

| Field | Meaning |
| --- | --- |
| `ethercat_master_index` | IgH master index passed to `ecrt_request_master`. |
| `alias` | EtherCAT slave alias. |
| `position` | EtherCAT ring position. |
| `vendor_id` | Slave vendor ID. |
| `product_id` | Slave product code. |
| `profile_mode` | `0`: position, `1`: velocity, `2`: effort. Used when setting operation mode and filtering target PDOs. |

## `EthercatMaster`

| Function | Description |
| --- | --- |
| `initialize()` | Requests the IgH master and creates a domain. |
| `activate()` | Activates the master and caches `ecrt_domain_data`. |
| `deactivate()` | Deactivates the IgH master. |
| `transmit()` | Queues the domain and sends datagrams. |
| `receive()` | Receives frames and processes the domain. |
| `apply_application_time(time)` | Converts `timespec` to nanoseconds and calls `ecrt_master_application_time`. |
| `save_clock()` | Calls `ecrt_master_sync_slave_clocks`. |
| `master()` / `domain()` / `domain_pd()` | Return IgH handles used by the controller. |

## `EthercatController`

| Function | Description |
| --- | --- |
| `initialize(master, driver)` | Casts the master, creates `ecrt_master_slave_config`, registers SDO/PDO entries, and configures distributed clocks. |
| `registerEntries()` | Runs SDO setup and PDO setup. |
| `enable()` | Reads statusword from the domain and writes the next driver controlword until the driver reports enabled. |
| `disable()` | Writes the disable transition controlword until the driver reports disabled. |
| `check(status)` | Handles set-point acknowledge through `driver_->isReceived`. |
| `write(command)` | Converts command fields to raw RX PDO entries and writes them into the domain. |
| `read(status)` | Reads TX PDO entries from the domain, applies driver scaling, and fills `motor_frame_t`. |

The controller supports target IDs `ID_CONTROLWORD`, `ID_TARGET_POSITION`, `ID_TARGET_VELOCITY`, and `ID_TARGET_EFFORT` on writes, and status/error/current position/current velocity/current effort IDs on reads.
