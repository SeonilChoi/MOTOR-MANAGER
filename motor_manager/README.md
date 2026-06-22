# motor_manager library

## English Version

`motor_manager::MotorManager` is a runtime library that loads motor configuration from a YAML file and creates and manages `master`, `controller`, and `driver` objects.

`EtherCAT`, `CANopen`, and `SocketCAN` masters are handled in the main loop. `Serial` masters are handled by separate serial threads.

### API

#### `CommunicationType`

`CommunicationType` consists of `Ethercat`, `Canopen`, `Socketcan`, and `Serial`.

#### `DriverType`

`DriverType` consists of `Minas`, `Zeroerr`, `Dynamixel`, and `Cubemars`.

### Public Functions

| Function | Purpose |
| --- | --- |
| `MotorManager(config_file)` | Loads the YAML configuration and initializes the manager. |
| `run()` | Runs the main loop and starts the serial loop. |
| `write(command, size)` | Stores external command frames in the internal command buffer. |
| `read(status)` | Reads motor status from the internal status buffer. |
| `request_stop()` | Requests all controllers to enter the `Disable` state. |
| `request_exit()` | Requests the `run()` loop to exit. |
| `period()` | Returns `period_`. |
| `number_of_controllers()` | Returns the number of controllers. |

### Internal Functions

| Function | Purpose |
| --- | --- |
| `loadConfigurations(config_file)` | Loads configuration from a YAML file. |
| `initialize()` | Initializes `master`, `driver`, and `controller` objects. |
| `start()` | Runs `master->activate()` for every master. |
| `stop()` | Runs `master->deactivate()` for every master. |
| `enableControllers(controller_indices)` | Runs `controller->enable()` for the given controller indices. |
| `disableControllers(controller_indices)` | Runs `controller->disable()` for the given controller indices. |
| `updateControllers(controller_indices)` | Reads status with `controller->read()`, checks status with `controller->check()`, and writes a pending command with `controller->write()` when a new command exists. |
| `refreshDisabled()` | Checks whether all controllers are in the `Disable` state and updates `is_disabled_`. |
| `startSerial()` | Starts the serial loop in separate threads. |
| `stopSerial()` | Stops the serial loop. |
| `serialRun(master_id)` | Runs one serial master loop. |
| `setSerialException(exception)` | Stores an exception raised from a serial loop. |
| `rethrowSerialExceptionIfAny()` | Rethrows a stored serial exception in the main loop. |

### Internal State

| Name | Purpose |
| --- | --- |
| `masters_` | Masters handled by the main loop. |
| `serial_masters_` | Serial masters handled by serial threads. |
| `drivers_` | Driver objects created from the YAML configuration. |
| `controllers_` | Controller objects indexed by `controller_index`. |
| `controller_indices_` | Controller indices handled by the main loop. |
| `serial_controller_indices_` | Controller indices handled by each serial master. |
| `command_` | Internal command buffer. |
| `status_` | Internal status buffer. |
| `command_sequence_` | Sequence counter updated whenever a new command is written. |
| `applied_command_sequence_` | Sequence counter for the last command applied to each controller. |

## Korean Version

`motor_manager::MotorManager`는 런타임 라이브러리로, YAML 파일을 통해 모터 설정을 불러오고 `master`, `controller`, `driver` 객체를 생성해 관리한다.

`EtherCAT`, `CANopen`, `SocketCAN` 마스터는 메인 루프에서 처리하고, `Serial` 마스터는 별도의 serial 스레드에서 처리한다.

### API

#### `CommunicationType`

`CommunicationType`은 `Ethercat`, `Canopen`, `Socketcan`, `Serial`로 구성된다.

#### `DriverType`

`DriverType`은 `Minas`, `Zeroerr`, `Dynamixel`, `Cubemars`로 구성된다.

### Public Functions

| Function | Purpose |
| --- | --- |
| `MotorManager(config_file)` | YAML configuration을 불러오고 manager를 초기화한다. |
| `run()` | 메인 루프를 실행하고 serial 루프를 시작한다. |
| `write(command, size)` | 외부 command frame을 내부 command buffer에 저장한다. |
| `read(status)` | 내부 status buffer에서 모터 상태를 읽는다. |
| `request_stop()` | 모든 controller가 `Disable` 상태가 되도록 요청한다. |
| `request_exit()` | `run()` 루프 종료를 요청한다. |
| `period()` | `period_`를 반환한다. |
| `number_of_controllers()` | `controller`의 수를 반환한다. |

### Internal Functions

| Function | Purpose |
| --- | --- |
| `loadConfigurations(config_file)` | YAML 파일을 읽어서 configuration을 불러온다. |
| `initialize()` | `master`, `driver`, `controller`를 초기화한다. |
| `start()` | 모든 `master->activate()`를 실행한다. |
| `stop()` | 모든 `master->deactivate()`를 실행한다. |
| `enableControllers(controller_indices)` | 전달받은 controller index 목록에 대해 `controller->enable()`을 실행한다. |
| `disableControllers(controller_indices)` | 전달받은 controller index 목록에 대해 `controller->disable()`을 실행한다. |
| `updateControllers(controller_indices)` | `controller->read()`로 상태를 읽고, `controller->check()`로 상태를 확인한 뒤, 새로운 command가 있으면 `controller->write()`로 명령을 보낸다. |
| `refreshDisabled()` | 모든 controller가 `Disable` 상태인지 확인하고 `is_disabled_`를 갱신한다. |
| `startSerial()` | serial 루프를 별도 스레드로 실행한다. |
| `stopSerial()` | serial 루프를 종료한다. |
| `serialRun(master_id)` | 하나의 serial master 루프를 실행한다. |
| `setSerialException(exception)` | serial 루프에서 발생한 exception을 저장한다. |
| `rethrowSerialExceptionIfAny()` | 저장된 serial exception이 있으면 메인 루프에서 다시 던진다. |

### Internal State

| Name | Purpose |
| --- | --- |
| `masters_` | 메인 루프에서 처리하는 master 객체 |
| `serial_masters_` | serial 스레드에서 처리하는 serial master 객체 |
| `drivers_` | YAML configuration으로 생성한 driver 객체 |
| `controllers_` | `controller_index`로 접근하는 controller 객체 |
| `controller_indices_` | 메인 루프에서 처리하는 controller index 목록 |
| `serial_controller_indices_` | 각 serial master가 처리하는 controller index 목록 |
| `command_` | 내부 command buffer |
| `status_` | 내부 status buffer |
| `command_sequence_` | 새로운 command가 들어올 때마다 갱신되는 sequence counter |
| `applied_command_sequence_` | 각 controller에 마지막으로 적용한 command sequence counter |
