# motor_manager

## Overview

EtherCAT motor stack as one `ament_cmake` package: abstract interfaces, EtherCAT transport, vendor drivers, and a YAML-driven `MotorManager` over `motor_frame_t` (`common_motor_interface`).

## Repository layout

```text
lib/motor_manager/
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

## Package READMEs

- [core/motor_interface](core/motor_interface/README.md)
- [communications/ethercat](communications/ethercat/README.md)
- [hardware/minas](hardware/minas/README.md)
- [hardware/zeroerr](hardware/zeroerr/README.md)
- [motor_manager (library)](motor_manager/README.md)
