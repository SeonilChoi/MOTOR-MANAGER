# minas

**`MinasDriver`**: Panasonic MINAS CiA402 mapping, parameter YAML loading, and unit conversion. Built into `motor_manager`; YAML `drivers[].type: "minas"`.

## `MinasDriver` functions

| Function | Description |
|----------|-------------|
| `MinasDriver(config)` | Forwards `driver_config_t` to `MotorDriver`; no extra state. |
| `loadParameters(param_file)` | Loads YAML `items` into `items_` (CoE values): special-case IDs fill limits, max torque, speed, and profile fields from `config_`; others use `value` by type. Loads `interfaces` into `interfaces_` and sets RX/TX counts from PDO entries vs `ID_RXPDO` / `ID_TXPDO` markers. Throws on bad YAML or unknown types. |
| `isEnabled(data, driver_state, out)` | CiA402-style state machine from `DriverState` and statusword in `data`; writes next controlword to `out`. Handles fault → fault reset. Returns `true` only in `OperationEnabled`. |
| `isDisabled(data, driver_state, out)` | Reverse sequence toward `SwitchOnDisabled`; writes controlword to `out`. Returns `true` when already `SwitchOnDisabled`. |
| `isReceived(data, out)` | If statusword has set-point acknowledge bit, writes `0x000F` to `out` and returns `true`; else `false`. |
| `position` / `velocity` / `torque` (raw ↔ physical) | Same formulas as `MotorDriver` contract: position/velocity use `pulse_per_revolution` and \(2\pi\) rad per rev; torque uses `rated_torque`, `unit_torque`, and 0.01% scaling. |

## Namespace constants (`minas`, header)

Semantic IDs for MINAS-specific YAML `items` / `interfaces` entries (see `loadParameters`):

| Name | Value | Role in `loadParameters` |
|------|-------|---------------------------|
| `ID_MAX_TORQUE` | 50 | Fills max torque SDO from `unit_torque`. |
| `ID_MIN_POSITION_LIMIT` | 51 | Lower position limit from `config_.lower` (rad → counts). |
| `ID_MAX_POSITION_LIMIT` | 52 | Upper limit from `config_.upper`. |
| `ID_MAX_MOTOR_SPEED` | 53 | From `config_.speed`. |
| `ID_PROFILE_VELOCITY` | 54 | From `profile_velocity` (rad/s → counts). |
| `ID_PROFILE_ACCELERATION` | 55 | From `profile_acceleration`. |
| `ID_PROFILE_DECELERATION` | 56 | From `profile_deceleration`. |
| `ID_MAX_ACCELERATION` | 57 | From `acceleration`. |
| `ID_MAX_DECELERATION` | 58 | From `deceleration`. |
| `ID_RXPDO` | 98 | Marks RX PDO container row in `interfaces` (no subindex/size in loop). |
| `ID_TXPDO` | 99 | Marks TX PDO container row. |

## File-local symbols (`src/minas_driver.cpp`)

Anonymous namespace — not part of the public API:

| Symbol | Description |
|--------|-------------|
| `CW_SHUTDOWN`, `CW_SWITCH_ON`, `CW_ENABLE_OPERATION`, `CW_DISABLE_VOLTAGE`, `CW_DISABLE_OPERATION`, `CW_FAULT_RESET` | Controlword bit patterns for MINAS CiA402 transitions (values differ from ZeroErr). |
| `isFault`, `isReadyToSwitchOn`, `isSwitchedOn`, `isOperationEnabled`, `isSwitchOnDisabled`, `isSetpointAcknowledge` | Statusword predicates shared by `isEnabled` / `isDisabled` / `isReceived`. |
