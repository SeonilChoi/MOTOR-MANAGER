# motor_manager (library)

`MotorManager` — **`run()`**, **`write()`**, **`read()`** over `motor_frame_t` (`include/motor_manager/motor_manager.hpp`).

---

## `run()`

```mermaid
flowchart LR
  P[setup + activate] --> A[sleep]
  A --> B[receive]
  B --> C[enable / update]
  C --> D[transmit]
  D --> A
  D -.->|stop| Q[deactivate]
```

`request_stop()` ends the loop; then masters deactivate and memory is unlocked.

---

## `write()` · `read()` · `update()`

```mermaid
flowchart TB
  W[write] -->|mutex| BUF["command_ + status_"]
  R[read] -->|mutex| BUF
  U[update in run] -->|mutex| BUF
  U --> BUS[EtherCAT in/out]
```

- **`write()`** / **`read()`**: other thread; only copy under **`frame_mutex_`** (plus **`is_command_changed_`** on write path).
- **`update()`**: inside **`run()`**; refreshes **`status_`**, may push **`command_`** to the domain after **`receive`**.
