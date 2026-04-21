# zeroerr

EtherCAT **ZeroErr** servo driver implementation: `ZeroerrDriver` (`include/zeroerr/zeroerr_driver.hpp`, `src/zeroerr_driver.cpp`). Built when `motor_manager` links this subdirectory; selected in motor YAML with `drivers[].type: "zeroerr"`.

## `ZeroerrDriver` functions

| Function | Description |
|----------|-------------|
| `ZeroerrDriver(config)` | Forwards `driver_config_t` to `MotorDriver`; no extra state. |
| `loadParameters(param_file)` | Loads YAML `items` into `items_`: special-case IDs fill position limits and profile velocity/accel/decel from `config_`; others use `value` by type. Loads `interfaces` and RX/TX counts like MINAS (PDO rows vs `ID_RXPDO` / `ID_TXPDO`). Throws on bad YAML or unknown types. |
| `isEnabled(data, driver_state, out)` | CiA402-style enable sequence; controlword constants use ZeroErr-specific values. Fault handling and `out` controlword same pattern as `MinasDriver::isEnabled`. |
| `isDisabled(data, driver_state, out)` | Disable sequence toward `SwitchOnDisabled`; same structure as MINAS with different `CW_*` literals. |
| `isReceived(data, out)` | Same set-point-acknowledge handling as MINAS (`0x000F` when bit set). |
| `position` / `velocity` / `torque` (raw ↔ physical) | Same scaling as `MinasDriver` (pulses per rev, \(2\pi\), rated torque / `unit_torque`). |

## Namespace constants (`zeroerr`, header)

Semantic IDs for ZeroErr YAML `items` / `interfaces`:

| Name | Value | Role in `loadParameters` |
|------|-------|---------------------------|
| `ID_MIN_POSITION_LIMIT` | 50 | Lower position limit from `config_.lower` (rad → counts). |
| `ID_MAX_POSITION_LIMIT` | 51 | Upper limit from `config_.upper`. |
| `ID_PROFILE_VELOCITY` | 52 | From `profile_velocity` (rad/s → counts). |
| `ID_PROFILE_ACCELERATION` | 53 | From `profile_acceleration`. |
| `ID_PROFILE_DECELERATION` | 54 | From `profile_deceleration`. |
| `ID_RXPDO` | 98 | RX PDO marker in `interfaces` list. |
| `ID_TXPDO` | 99 | TX PDO marker. |

## File-local symbols (`src/zeroerr_driver.cpp`)

Anonymous namespace — not part of the public API:

| Symbol | Description |
|--------|-------------|
| `CW_SHUTDOWN`, `CW_SWITCH_ON`, `CW_ENABLE_OPERATION`, `CW_DISABLE_VOLTAGE`, `CW_DISABLE_OPERATION`, `CW_FAULT_RESET` | ZeroErr-specific controlword values for CiA402 transitions (e.g. shutdown/switch-on differ from MINAS). |
| `isFault`, `isReadyToSwitchOn`, `isSwitchedOn`, `isOperationEnabled`, `isSwitchOnDisabled`, `isSetpointAcknowledge` | Statusword predicates for enable/disable/receive (`isSwitchOnDisabled` uses a different mask than the MINAS driver). |
