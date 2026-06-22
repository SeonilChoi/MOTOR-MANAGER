# motor_manager library

`motor_manager::MotorManager` is the runtime library that loads a motor YAML file, constructs masters/drivers/controllers, and runs the cyclic bus loop.

Header: `include/motor_manager/motor_manager.hpp`

## Construction

```cpp
#include "motor_manager/motor_manager.hpp"

motor_manager::MotorManager manager("/path/to/config.yaml");
manager.run();
```

The constructor loads the YAML configuration and initializes all masters and controllers. `run()` activates the masters and blocks until `request_stop()` finishes disabling all axes or `request_exit()` clears the loop flag.

## Public API

| Function | Description |
| --- | --- |
| `MotorManager(config_file)` | Loads YAML, creates EtherCAT/CANopen masters, creates MINAS/ZeroErr drivers, loads driver parameter YAML, and initializes controllers. |
| `run()` | Starts the periodic loop. Requires permission for `mlockall` and `SCHED_FIFO`. |
| `write(command, size)` | Copies command frames into the manager and marks the command buffer changed. |
| `read(status)` | Copies the latest status frames out of the manager. |
| `request_stop()` | Requests drive disable; the loop returns after all controllers report disabled. |
| `request_exit()` | Stops the loop without waiting for drive disable. |
| `period()` | Returns the configured period in nanoseconds. |
| `number_of_controllers()` | Returns the number of configured slave/controller entries. |

The controller update path reads every controller, runs each controller's set-point check, stops command output if any status frame has a nonzero `errorcode`, and writes new commands only when `write()` has provided a newer command frame.

## Configuration schema

Top-level keys:

| Key | Meaning |
| --- | --- |
| `period` | Cycle period in nanoseconds. |
| `masters` | List of fieldbus masters and their slaves/nodes. |
| `drivers` | List of vendor driver configs. |

EtherCAT, CANopen, and SocketCAN masters run in the real-time loop. Serial
masters run in separate worker threads so blocking Dynamixel traffic does not
delay the EtherCAT cycle.

Supported values in the current code:

| Field | Supported values |
| --- | --- |
| `masters[].type` | `ethercat`, `canopen`, `socketcan`, `serial`, `dynamixel` |
| `drivers[].type` | `minas`, `zeroerr`, `dynamixel`, `cubemars` |
| `profile_mode` | `0`: position, `1`: velocity, `2`: effort |

`param_file` can point to a YAML file or to a directory. YAML files are loaded directly; directories load `<param_file>/<type>.yaml`. Relative paths are resolved from the main config file.

## Threading

`write()` and `read()` share `command_` and `status_` through `frame_mutex_`. The real-time loop and serial workers copy command/status snapshots under that mutex, then perform controller and bus work outside the lock.

`request_stop()` and `request_exit()` use atomics and can be called from another thread.
