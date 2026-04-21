# ethercat

IgH EtherCAT integration: **`EthercatMaster`** (`MotorMaster`) and **`EthercatController`** (`MotorController`) under namespace `ethercat`. Maps PDOs and **`motor_frame_t`** to the domain process image.

## `EthercatMaster`

| Function | Description |
|----------|-------------|
| `initialize()` | Requests the IgH master (`ecrt_request_master`), creates a domain (`ecrt_master_create_domain`). Throws if either fails. |
| `activate()` | Activates the master and caches the domain process-data pointer (`ecrt_domain_data`). Throws on failure. |
| `deactivate()` | Deactivates the master (`ecrt_master_deactivate`). Throws on failure. |
| `transmit()` | Queues domain datagrams and sends frames (`ecrt_domain_queue`, `ecrt_master_send`). Throws on failure. |
| `receive()` | Receives frames and processes the domain (`ecrt_master_receive`, `ecrt_domain_process`). Throws on failure. |
| `apply_application_time(time)` | Converts `timespec` to nanoseconds and calls `ecrt_master_application_time` (distributed clock / app time). |
| `save_clock()` | Calls `ecrt_master_sync_slave_clocks`. |
| `master()` | Returns the `ec_master_t*` handle. |
| `domain()` | Returns the `ec_domain_t*` handle. |
| `domain_pd()` | Returns the domain process-image base pointer used for `EC_READ_*` / `EC_WRITE_*`. |
| `master_index()` | IgH master index from `master_config_t`. |

## `EthercatController`

| Function | Description |
|----------|-------------|
| `initialize(master, driver)` | Casts `master` to `EthercatMaster`, creates `ecrt_master_slave_config` for alias/position/vendor/product, then registers PDO/SDO layout. Throws if the cast or slave config fails. |
| `registerEntries()` | Runs SDO download from the driver’s `items_` (`addSlaveConfigSdos`) and PDO registration / offsets (`addSlaveConfigPdos`). |
| `enable()` | Reads statusword from the domain, asks the driver for the next controlword step (`isEnabled`); writes controlword to the domain until the sequence reports done. Returns `true` when enabled, `false` while stepping. |
| `disable()` | Same pattern as `enable()` using `isDisabled`. |
| `check(status)` | If `driver_->isReceived` accepts the given `status.statusword`, writes the resulting controlword into the domain. |
| `write(command)` | Builds RX `entry_table_t` rows from `command.target_interface_id` (controlword, target position/velocity/torque with driver scaling), then `writeData` into the domain. Throws on unknown RX id. |
| `read(status)` | `readData` from the domain into TX buffers, then fills `statusword`, `errorcode`, `position`, `velocity`, `torque` (driver de-scaling) and sets `controller_index`. Throws on unknown TX id. |

Internal: `writeData` / `readData` map `entry_table_t` types to `EC_WRITE_*` / `EC_READ_*` at `offset_[id]`; `addSlaveConfigSdos` applies CoE SDOs from driver items; `addSlaveConfigPdos` builds sync/PDO layout, registers entries with the domain, and fills `tx_interfaces_` / `offset_`.
