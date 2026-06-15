# Motor Manager

<p align="left">
  <img src="images/banner.gif" alt="EtherCAT simultaneous control demo with Panasonic Minas actuators and Zeroerr eRob motors">
</p>
<p align="left"><em>Execution result: simultaneous EtherCAT control of two Panasonic Minas actuators and three Zeroerr eRob motors.</em></p>

## Overview

EtherCAT motor stack as one `ament_cmake` package: abstract interfaces, EtherCAT transport, vendor drivers, and a YAML-driven `MotorManager` over `motor_frame_t` (`common_motor_interface`).

## Clone

```bash
mkdir motor_manager
git clone https://github.com/SeonilChoi/MOTOR-MANAGER.git ./motor_manager
```

## Setup

#### EtherCAT
```bash ethercat
sudo ethercatctl restart
sudo chmod 777 /dev/EtherCAT0
```

#### CANopen
```bash canopen
sudo slcand -o -c -s8 /dev/CANable can0
sudo ip link set can0 up
```

## Repository layout

```text
motor_manager/
├── CMakeLists.txt
├── package.xml
├── README.md
├── core/
│   └── motor_interface/
│       ├── CMakeLists.txt
│       ├── README.md
│       └── include/motor_interface/
├── communications/
│   └── ethercat/
│       ├── CMakeLists.txt
│       ├── README.md
│       ├── include/ethercat/
│       └── src/
├── hardware/
│   ├── minas/
│   │   ├── CMakeLists.txt
│   │   ├── README.md
│   │   ├── include/minas/
│   │   └── src/
│   └── zeroerr/
│       ├── CMakeLists.txt
│       ├── README.md
│       ├── include/zeroerr/
│       └── src/
└── motor_manager/
    ├── CMakeLists.txt
    ├── README.md
    ├── include/motor_manager/
    └── src/
```
