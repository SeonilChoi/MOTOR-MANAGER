# Motor Manager

<p align="left">
  <img src="images/banner.gif" alt="EtherCAT simultaneous control demo with Panasonic Minas actuators and Zeroerr eRob motors">
</p>

`motor_manager` is an `ament_cmake` C++ library package for cyclic motor control. It contains the abstract motor interfaces, EtherCAT and CANopen transports, MINAS and ZeroErr driver mappings, and the YAML-driven `motor_manager::MotorManager` runtime.

## Packages and libraries

This repository is built as one ROS 2 package named `motor_manager`.

| Path | Library | Purpose |
| --- | --- | --- |
| `core/motor_interface` | `motor_interface` | Common `MotorMaster`, `MotorController`, and `MotorDriver` abstractions. |
| `communications/ethercat` | `ethercat` | IgH EtherCAT master/controller implementation. |
| `communications/canopen` | `canopen` | SocketCAN CANopen master/controller implementation. |
| `communications/serial` | `serial` | Dynamixel Protocol 2.0 serial master/controller implementation. |
| `communications/socketcan` | `socketcan` | Raw SocketCAN master/controller implementation. |
| `hardware/minas` | `minas` | Panasonic MINAS CiA402 driver mapping. |
| `hardware/zeroerr` | `zeroerr` | ZeroErr CiA402 driver mapping. |
| `hardware/dynamixel` | `dynamixel` | Dynamixel driver mapping. |
| `hardware/cubemars` | `cubemars` | CubeMars driver mapping. |
| `motor_manager` | `motor_manager` | YAML configuration loader and cyclic runtime loop. |

## Build

`motor_manager` depends on:

- ROS 2 with `ament_cmake`
- `common_motor_interface`
- `yaml-cpp`
- IgH EtherCAT headers and library: `ecrt.h` and `libethercat`

From the root of a colcon workspace:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to motor_manager
source install/setup.bash
```

If IgH EtherCAT is installed in a non-standard path, set:

```bash
colcon build --packages-up-to motor_manager --cmake-args \
  -DMOTOR_MANAGER_IGH_ETHERCAT_INCLUDE_DIR=/path/to/include \
  -DMOTOR_MANAGER_IGH_ETHERCAT_LIB=/path/to/libethercat.so
```

## Device setup

### EtherCAT

```bash
sudo ethercatctl restart
sudo chmod 666 /dev/EtherCAT0
```

### CANopen

Bring up a SocketCAN interface before starting the manager. For example, with a CANable adapter:

```bash
sudo slcand -o -c -s8 -S 3000000 /dev/CANable can0
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up
```

The current CANopen implementation binds to `can<can_interface_index>` from the YAML configuration.

## Configuration

`MotorManager` loads one YAML file. The top-level keys are `period`, `masters`, and `drivers`.
EtherCAT, CANopen, and SocketCAN masters run in the real-time loop. Serial
masters run in separate worker threads so blocking Dynamixel traffic does not
delay the EtherCAT cycle.

```yaml
period: 1000000
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
drivers:
  - id: 0
    type: zeroerr
    pulse_per_revolution: 524288
    rated_torque: 52.0
    unit_torque: 0.1
    lower: -1.0
    upper: 1.0
    speed: 3000
    acceleration: 2.5
    deceleration: 2.5
    profile_velocity: 2.5
    profile_acceleration: 2.5
    profile_deceleration: 2.5
    profile_position_value: 1
    profile_velocity_value: 3
    profile_torque_value: 4
    param_file: ../param
```

`profile_mode` selects which command PDO is kept active:

| Value | Mode |
| --- | --- |
| `0` | Profile position |
| `1` | Profile velocity |
| `2` | Profile torque |

For each driver, `param_file` can point to a YAML file or to a directory. YAML files are loaded directly; directories load `<param_file>/<type>.yaml`. Relative paths are resolved from the main config file.

Examples in this workspace:

- `src/ros2/motion_system_ros2/ros2_motor_manager/config/example_ethercat_zeroerr.yaml`
- `src/ros2/motion_system_ros2/ros2_motor_manager/config/example_canopen_zeroerr.yaml`
- `src/ros2/motion_system_ros2/ros2_motor_manager/param/zeroerr.yaml`

## Runtime API

`motor_manager::MotorManager` is constructed with a config file path:

```cpp
motor_manager::MotorManager manager(config_file);
manager.run();
```

The real-time cyclic loop:

1. Activates every master.
2. Starts serial worker threads for serial masters.
3. Locks memory and requests `SCHED_FIFO`.
4. Sleeps at the configured nanosecond `period`.
5. Receives non-serial bus data.
6. Enables drives, applies command updates, or disables drives when requested.
7. Syncs clocks where supported.
8. Transmits non-serial bus data.

Commands and status use `motor_interface::motor_frame_t` from `common_motor_interface`.

| Function | Purpose |
| --- | --- |
| `run()` | Starts the real-time cyclic loop and blocks until exit/disable. |
| `write(command, size)` | Copies up to `MAX_CONTROLLER_SIZE` command frames into the manager. |
| `read(status)` | Copies the latest status frames out of the manager. |
| `request_stop()` | Requests CiA402 disable and lets `run()` return after all axes report disabled. |
| `request_exit()` | Stops the loop without waiting for drive disable. |
| `period()` | Returns the configured cycle period in nanoseconds. |
| `number_of_controllers()` | Returns the number of configured controllers. |

## Layout

```text
motor_manager/
├── CMakeLists.txt
├── package.xml
├── communications/
│   ├── canopen/
│   └── ethercat/
├── core/
│   └── motor_interface/
├── hardware/
│   ├── minas/
│   └── zeroerr/
└── motor_manager/
```
