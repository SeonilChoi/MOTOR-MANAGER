# motor_interface

Abstract C++ interfaces used by the `motor_manager` package. The headers are installed from `include/motor_interface` and all symbols live in namespace `motor_interface`.

This library does not talk to hardware directly. Concrete transports implement `MotorMaster` and `MotorController`; concrete vendors implement `MotorDriver`. Transport-specific driver hooks live outside this package, such as `serial::SerialDriver` and `socketcan::SocketcanDriver`.

## Limits

| Constant | Value | Meaning |
| --- | --- | --- |
| `MAX_MASTER_SIZE` | `8` | Maximum configured masters. |
| `MAX_DRIVER_SIZE` | `8` | Maximum configured drivers. |
| `MAX_CONTROLLER_SIZE` | `16` | Maximum configured controllers. |
| `MAX_DATA_SIZE` | `4` | Maximum bytes stored in `entry_table_t::data`. |
| `MAX_ITEM_SIZE` | `32` | Maximum SDO/config items per driver. |

## `MotorMaster`

Declared in `include/motor_interface/motor_master.hpp`.

`MotorMaster` represents one fieldbus master. EtherCAT and CANopen implementations derive from it.

### `master_config_t`

| Field | Type | Used by | Meaning |
| --- | --- | --- | --- |
| `id` | `uint8_t` | all | Master ID from `masters[].id`. |
| `number_of_slaves` | `uint8_t` | all | Number of slave/controller entries under the master. |
| `ethercat_master_index` | `unsigned int` | EtherCAT | IgH master index passed to `ecrt_request_master`. |
| `can_interface_index` | `unsigned int` | CANopen | SocketCAN suffix; `0` binds to `can0`. |
| `can_bitrate` | `unsigned int` | CANopen | Config value retained by the master. Interface bitrate must still be set before launch. |

### Virtual functions

| Function | Meaning |
| --- | --- |
| `initialize()` | Open/configure the master resource. |
| `activate()` | Start communication. |
| `deactivate()` | Stop communication and release resources. |
| `transmit()` | Send pending bus data. |
| `receive()` | Receive/process bus data. |
| `apply_application_time(time)` | Apply cycle timestamp when the bus supports it. |
| `save_clock()` | Synchronize/save bus clocks when supported. |

## `MotorController`

Declared in `include/motor_interface/motor_controller.hpp`.

`MotorController` maps one configured slave/node to one `MotorMaster` and one `MotorDriver`.

### `slave_config_t`

| Field | Type | Used by | Meaning |
| --- | --- | --- | --- |
| `controller_index` | `uint8_t` | all | Dense index in `MotorManager::controllers_`. |
| `master_id` | `uint8_t` | all | Owning master ID. |
| `driver_id` | `uint8_t` | all | Driver ID used for PDO layout and scaling. |
| `alias` | `uint16_t` | EtherCAT | EtherCAT alias. |
| `position` | `uint16_t` | EtherCAT | EtherCAT ring position. |
| `can_id` | `uint8_t` | CANopen / SocketCAN / Serial | Bus node or CAN ID used by the controller. |
| `vendor_id` | `uint32_t` | EtherCAT | EtherCAT vendor ID. |
| `product_id` | `uint32_t` | EtherCAT | EtherCAT product code. |
| `profile_mode` | `int8_t` | all controllers | `0`: position, `1`: velocity, `2`: torque. |

The concrete controller implements `enable`, `disable`, `check`, `write`, and `read` over `motor_frame_t`.

## `MotorDriver`

Declared in `include/motor_interface/motor_driver.hpp`.

`MotorDriver` owns vendor-specific SDO items, PDO interface definitions, CiA402 enable/disable transitions, and unit conversion between raw drive values and physical values.

### `driver_config_t`

| Field | Type | Meaning |
| --- | --- | --- |
| `id` | `uint8_t` | Driver ID from `drivers[].id`. |
| `pulse_per_revolution` | `uint32_t` | Encoder scale for position/velocity conversion. |
| `rated_torque` | `double` | Rated torque used by torque conversion. |
| `unit_torque` | `double` | Raw torque unit scale. |
| `lower` / `upper` | `double` | Software position limits. |
| `speed` | `double` | Maximum speed parameter. |
| `acceleration` / `deceleration` | `double` | Maximum acceleration/deceleration parameters. |
| `profile_velocity` | `double` | Profile velocity parameter. |
| `profile_acceleration` | `double` | Profile acceleration parameter. |
| `profile_deceleration` | `double` | Profile deceleration parameter. |
| `profile_position_value` | `int8_t` | Operation-mode value for profile position mode. |
| `profile_velocity_value` | `int8_t` | Operation-mode value for profile velocity mode. |
| `profile_torque_value` | `int8_t` | Operation-mode value for profile torque mode. |

### `entry_table_t`

| Field | Type | Meaning |
| --- | --- | --- |
| `id` | `uint8_t` | Semantic interface/config ID. |
| `index` | `uint16_t` | CANopen object dictionary index. |
| `subindex` | `uint8_t` | CANopen object dictionary subindex. |
| `type` | `DataType` | Raw data type. |
| `size` | `uint8_t` | Payload size in bytes. |
| `data` | `uint8_t[MAX_DATA_SIZE]` | Little-endian payload buffer. |

### Enums

`DataType`: `U8`, `U16`, `U32`, `U64`, `S8`, `S16`, `S32`

`DriverState`: `Fault`, `SwitchOnDisabled`, `ReadyToSwitchOn`, `SwitchedOn`, `OperationEnabled`

### Helper functions

| Function | Purpose |
| --- | --- |
| `DataType toDataType(const std::string& type)` | Parses YAML strings such as `u16` or `s32`. |
| `template <typename T> T value(const uint8_t* data)` | Decodes a little-endian byte buffer. |
| `template <typename T> void fill(const T& value, uint8_t* data)` | Encodes a value into a little-endian byte buffer. |

## Semantic IDs

These IDs are used by driver YAML files and `motor_frame_t::target_interface_id`.

| Constant | Value | Meaning |
| --- | --- | --- |
| `ID_CONTROLWORD` | `0` | Command controlword. |
| `ID_TARGET_POSITION` | `1` | Target position. |
| `ID_TARGET_VELOCITY` | `2` | Target velocity. |
| `ID_TARGET_TORQUE` | `3` | Target torque. |
| `ID_STATUSWORD` | `4` | Statusword. |
| `ID_ERRORCODE` | `5` | Error code. |
| `ID_CURRENT_POSITION` | `6` | Actual position. |
| `ID_CURRENT_VELOCITY` | `7` | Actual velocity. |
| `ID_CURRENT_TORQUE` | `8` | Actual torque. |
| `ID_OPERATING_MODE` | `30` | Operation mode object. |
| `ID_MAX_TORQUE` | `50` | Max torque parameter. |
| `ID_MIN_POSITION_LIMIT` | `51` | Minimum software position limit. |
| `ID_MAX_POSITION_LIMIT` | `52` | Maximum software position limit. |
| `ID_MAX_MOTOR_SPEED` | `53` | Maximum motor speed. |
| `ID_PROFILE_VELOCITY` | `54` | Profile velocity. |
| `ID_PROFILE_ACCELERATION` | `55` | Profile acceleration. |
| `ID_PROFILE_DECELERATION` | `56` | Profile deceleration. |
| `ID_MAX_ACCELERATION` | `57` | Maximum acceleration. |
| `ID_MAX_DECELERATION` | `58` | Maximum deceleration. |
| `ID_RXPDO` | `98` | RX PDO marker row in driver interface YAML. |
| `ID_TXPDO` | `99` | TX PDO marker row in driver interface YAML. |

# motor_interface

모터 제어를 위한 추상화 클래스로, `MotorMaster`, `MotorController`, `MotorDriver`로 구성된다.

## `MotorMaster`

`MotorMaster`는 하나의 필드버스 마스터를 담는다.

### `MAX_MASTER_SIZE`

| Name | Type | Meaning | Default |
| --- | --- | --- | --- |
| `MAX_MASTER_SIZE` | `uint8_t` | 가능한 마스터 수의 최대값 | `8` |

### `master_config_t`

`master_config_t`는 마스터를 초기화 할 때 사용하며, 마스터의 정보를 담는다.

| Name | Type | Meaning | Default |
| --- | --- | --- | --- |
| `id` | `uint8_t` | 마스터 ID | `0` |
| `number_of_slaves` | `uint8_t` | 마스터가 가질 `controller`의 수 | `0` |
| `ethercat_master_index` | `unsigned int` | EtherCAT 전용 마스터 인덱스 | `0` |
| `can_interface_index` | `unsigned int` | CAN 전용 장치 인덱스 | `0` |
| `can_bitrate` | `unsigned int` | CAN 전용 비트레이트 | `0` |
| `serial_port` | `std::string` | Serial 전용 포트 이름 | ` ` |
| `serial_baudrate` | `unsigned int` | Serial 전용 보드레이트 | `0` |

### functions

| Function | Purpose |
| --- | --- |
| `initialize()` | 마스터를 초기화한다. |
| `activate()` | 통신을 시작한다. |
| `deactivate()` | 통신을 멈춘다. |
| `transmit()` | 버스 데이터를 전송한다. |
| `receive()` | 버스 데이터를 수신한다. |
| `apply_application_time(time)` | 사이클 타임스텝을 적용한다. |
| `save_clock()` | 버스 시간을 동기화 및 저장한다. |
| `id()` | 마스터 ID를 반환한다. |
| `number_of_slaves()` | 마스터가 가진 슬레이브 수를 반환한다. |

## `MotorController`

`MotorController`는 슬레이브 혹은 노드를 담으며, 하나의 마스터와 하나의 드라이브를 갖는다.

### `MAX_CONTROLLER_SIZE`

| Name | Type | Meaning | Default |
| --- | --- | --- | --- |
| `MAX_CONTROLLER_SIZE` | `uint8_t` | 가능한 컨트롤러 수의 최대값 | `32` |

### `slave_config_t`

`slave_config_t`는 컨트롤러를 초기화 할 때 사용하며, 컨트롤러러의 정보를 담는다.

| Name | Type | Meaning | Default |
| --- | --- | --- | --- |
| `controller_index` | `uint8_t` | `motor_controller` 인덱스 | `0` |
| `master_id` | `uint8_t` | `motor_controller`가 속한 마스터 ID | `0` |
| `driver_id` | `uint8_t` | `motor_controller`의 드라이버 ID | `0` |
| `alias` | `uint16_t` | EtherCAT 전용 alias 값 | `0` |
| `position` | `uint16_t` | EtherCAT 전용 position 값 | `0` |
| `vendor_id` | `uint32_t` | EtherCAT 전용 vendor ID 값 | `0` |
| `product_id` | `uint32_t` | EtherCAT 전용 product ID 값 | `0` |
| `can_id` | `uint8_t` | CAN 전용 ID 값 | `0` |
| `profile_mode` | `uint8_t` | 0: 위치, 1: 속도, 2: 힘 | `0` |

### functions

| Function | Purpose |
| --- | --- |
| `initialize()` | 컨트롤러를 초기화 한다. |
| `enable()` | 모터를 `Enable` 상태로 만든다. |
| `disable()` | 모터를 `Disable` 상태로 만든다. |
| `check(status)` | 현재 모터 상태를 검사한다. |
| `write(command)` | 모터에 명령을 쓴다. |
| `read(status)` | 모터의 상태를 읽는다. |
| `master_id()` | 마스터 ID를 반환한다. |
| `driver_id()` | 드라이버 ID를 반환한다. |
| `registerEntries()` | 아이템과 인터페이스를 등록한다. |
| `writeData(rx_interfaces, number_of_rx_interfaces)` | 등록된 인터페이스에 데이터를 쓴다. |
| `readData(tx_interfaces, number_of_tx_interfaces)` | 등록된 인터페이스의 데이터를 읽는다. |

## `MotorDriver`

`MotorDriver`는 하드웨어 정보를 담으며, 제조사가 제공하는 하드웨어 제어 로직을 `MotorController`와 분리하는 역할을 한다.

### `MAX_DRIVER_SIZE`

| Name | Type | Meaning | Default |
| --- | --- | --- | --- |
| `MAX_DRIVER_SIZE` | `uint8_t` | 가능한 드라이버 수의 최대값 | `8` |

### `entry_table_t`

`entry_table_t`는 모터에 설정할 아이템이나 등록한 인터페이스의 정보를 담는 인터페이스이다.

| Name | Type | Meaning | Default |
| --- | --- | --- | --- |
| `MAX_DATA_SIZE` | `uint8_t` | 데이터의 최대 사이즈 (e.g. 4 bytes) | `4` |
| Name | Type | Meaning | Default |
| `DataType` | `class` | 데이터 타입을 의미하는 enum 클래스 (e.g. U32) | ` ` |
| Name | Type | Meaning | Default |
| `id` | `uint8_t` | 고유한 엔트리 ID | `0` |
| `index` | `uint16_t` | 엔트리 인덱스 (e.g 0x6040) | `0` |
| `subindex` | `uint8_t` | 엔트리 서브 인덱스 (e.g. 0x01) | ` ` |
| `type` | `DataType` | 엔트리 데이터 타입 | ` ` |
| `size` | `uint8_t` | 엔트리 데이터 사이즈 | ` ` |
| `data` | `uint8_t[MAX_DATA_SIZE]` | 엔트리 데이터 | ` ` |

### `MAX_ITEM_SIZE`

| Name | Type | Meaning | Default |
| --- | --- | --- | --- |
| `MAX_ITEM_SIZE` | `uint8_t` | 모터에 설정 가능한 아이템 수의 최대값 | `32` |

> [!NOTE]
> `item`은 SDO등을 활용해 초기화 단계에서 모터를 설정하는데 필요한 엔트리를 의미하며, `interface`는 PDO등을 활용해 지속적으로 송수신할 엔트리를 의미한다.