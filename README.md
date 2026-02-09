# Motor Manager

A C++ library for unified control of multiple motor drivers over different communication buses. It abstracts **masters** (bus interfaces), **drivers** (motor/amplifier protocols), and **controllers** (per-slave logic) so you can mix EtherCAT, CANopen, and Dynamixel in one application.

## Architecture

- **MotorMaster** – Owns a communication bus: init, activate/deactivate, transmit/receive. One master can have multiple slaves.
- **MotorDriver** – Protocol for a specific motor type: units (position/velocity/torque), status/controlword handling, PDO/object mappings. Holds `entry_table_t` items (config) and entries (Rx/Tx).
- **MotorController** – Binds one slave on a master to one driver: init, configure, servo on/off, read/write `motor_state_t`, and low-level `write_data`/`read_data` for PDOs or equivalent.

The main **MotorManager** loads YAML config, builds masters/drivers/controllers, and runs a cyclic update: receive → (optional enable/check/write/read) → transmit. Commands and states use a common **motor_state_t** (controlword, statusword, position, velocity, torque, etc.).

## Current Implementation

| Layer        | Implemented | Notes |
|-------------|-------------|--------|
| **Master**  | EtherCAT only | `EthercatMaster` (IgH Etherlab stack), `ecrt.h`, domain PDO |
| **Driver**  | Minas only   | `MinasDriver` – Panasonic Minas A6, CiA state machine, SDO/PDO via `entry_table_t` |
| **Controller** | EtherCAT only | `EthercatController` – slave config, SDO/PDO setup, alias/position, VID/PID |

Config: `config/example.yaml` (period, masters with type/slaves, drivers with type and `param_file`). Driver params in `param/` (e.g. `minas_a6.yaml`: objects and Rx/Tx entries with index/subindex/type).

## Types and Config

- **CommunicationType** (in `types.hpp`): `Ethercat`, `Canopen`, `Dynamixel` (already defined; only EtherCAT is implemented).
- **DriverType**: `Minas`, `Zeroerr`, `Dynamxiel` (only Minas implemented).
- **motor_state_t**: controlword, statusword, errorcode, position, velocity, torque; **entry_table_t**: index/subindex/type/size for SDO and PDO-style data.

## Adding CANopen and Dynamixel

### CANopen

- **Master**: Add `CanopenMaster` (e.g. using a CANopen stack or socketCAN). Implement `initialize`, `activate`, `deactivate`, `check`, `transmit`, `receive` (e.g. NMT, SDO, PDO sync/cyclic).
- **Controller**: Add `CanopenController` – map a node ID to a driver; implement `initialize`, `configure`, `servo_on`/`servo_off`, `check`/`write`/`read`, and `write_data`/`read_data` using the same `entry_table_t` (object index/subindex) so existing driver logic (e.g. Minas) can be reused where the object dictionary matches.
- **Config**: In `load_configurations`, handle `type: canopen` for masters (e.g. CAN interface, bitrate) and create `CanopenMaster` + `CanopenController`; slave config may use node_id instead of EtherCAT alias/position.

### Dynamixel

- **Master**: Add `DynamixelMaster` – serial or USB (e.g. USB2Dynamixel). Implement open/close, baudrate, and a single “transmit/receive” that sends Dynamixel packets (instruction + params) and reads status packets.
- **Driver**: Add `DynamixelDriver` (or reuse/extend a generic driver) – map Dynamixel control table addresses to the same interface IDs (controlword, target/current position/velocity/torque, statusword, errorcode) and implement `position`/`velocity`/`torque` conversions and `is_enabled`/`is_disabled`/`is_received` if needed.
- **Controller**: Add `DynamixelController` – one motor per ID; `write_data`/`read_data` become read/write of Dynamixel registers (e.g. goal position, present position, torque enable). No SDO/PDO; use instruction packets (READ, WRITE, etc.).
- **Config**: In `load_configurations`, handle `type: dynamixel` for masters (e.g. port, baudrate) and `type: dynamixel` for drivers; slave config may be minimal (e.g. just driver_id and Dynamixel ID).

### Integration in MotorManager

- In `motor_manager.cpp`, extend the `switch (to_communication_type(...))` in `load_configurations` with `case CommunicationType::Canopen:` and `case CommunicationType::Dynamixel:` to instantiate the corresponding master and controller types.
- Keep using the same `motor_state_t` and driver interfaces so higher-level code (update loop, enable/disable, check, read/write) stays unchanged; only the backend (EtherCAT vs CANopen vs Dynamixel) changes per master/controller.

## Build

- Dependencies: **yaml-cpp**, **IgH EtherCAT** (libethercat, for current code).
- CANopen/Dynamixel will add their own (e.g. CAN socket, serial library).
- CMake: `motor_manager_library` (shared) + `motor_manager_test`; link EtherCAT and yaml-cpp.

**Build and install:**

```bash
mkdir -p build
cd build
cmake -S .. -B . -DCMAKE_BUILD_TYPE=Release   # use Debug for debug build
sudo cmake --build . --target install
```

**Run the test executable** (from the `build` directory):

```bash
./motor_manager_test
```

## Summary

Motor Manager provides a single API to drive multiple motors over different buses. Today it supports **EtherCAT** with **Minas** drivers. The design already includes **CANopen** and **Dynamixel** in the type system; adding them means implementing the corresponding **MotorMaster** and **MotorController** (and for Dynamixel, a **MotorDriver**), then wiring them in `load_configurations` and CMake.
