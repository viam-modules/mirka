# Module mirka

This repository houses the Viam module for Mirka robotic sanding hardware.

## Model viam:mirka:airos-550cv

Drives a Mirka AIROS 550CV orbital sander through its motor drive cabinet over
Modbus RTU. Reports spindle speed, tool and drive temperatures, and decoded alarm
flags, and accepts start, stop and speed commands.

The drive's ON/OFF state is a hardware line (DI1), so this model controls
RUN/STOP and the speed setpoint only.

### Configuration for airos-550cv model

The following attribute template can be used to configure this model:

```json
{
    "serial_path": <string>,
    "baud_rate": <uint>,
    "parity": <string>,
    "slave_address": <uint8>,
    "default_rpm": <uint16>,
    "timeout_ms": <uint>
}
```

#### Attributes

The following attributes are available for this model:

| Name            | Type   | Inclusion | Description                                                                                      | Default |
| --------------- | ------ | --------- | ------------------------------------------------------------------------------------------------ | ------- |
| `serial_path`   | string | Required  | Serial device for the drive cabinet, e.g. `/dev/ttyUSB0`.                                          |         |
| `baud_rate`     | uint   | Optional  | Modbus RTU baud rate.                                                                              | 19200   |
| `parity`        | string | Optional  | Serial parity, one of `E`, `N`, `O`.                                                               | `E`     |
| `slave_address` | uint8  | Optional  | Modbus unit ID of the drive cabinet.                                                               | 86      |
| `default_rpm`   | uint16 | Optional  | Speed used by `start` when no `rpm` is given. Must be within 4000–10000.                           | 8000    |
| `timeout_ms`    | uint   | Optional  | Modbus request timeout.                                                                            | 1000    |

Speed is limited to **4000–10000 RPM**. `start` and `set_speed` reject values
outside that range.

#### Example Configuration

```json
{
    "serial_path": "/dev/ttyUSB0",
    "default_rpm": 8000
}
```

### DoCommand

Every command is a map with a `command` key naming the operation, plus that
command's parameters alongside it:

```json
{ "command": "<name>", ... }
```

#### start DoCommand

Starts the spindle. Uses `default_rpm` unless `rpm` is supplied.

```json
{ "command": "start", "rpm": 6000 }
```

Response:

```json
{ "running": true, "rpm": 6000 }
```

#### stop DoCommand

Stops the spindle.

```json
{ "command": "stop" }
```

Response:

```json
{ "running": false }
```

#### set_speed DoCommand

Writes the speed setpoint without changing the run state. `rpm` is required.

```json
{ "command": "set_speed", "rpm": 7500 }
```

Response:

```json
{ "rpm": 7500 }
```

#### status DoCommand

Returns the same payload as `GetStatus`.

```json
{ "command": "status" }
```

Response:

```json
{
    "running": true,
    "speed_setpoint_rpm": 8000,
    "average_speed_rpm": 7980,
    "tool_temp_c": 41,
    "drive_temp_c": 38,
    "operation_state": "RUN",
    "alarm_status": 0,
    "alarm_flags": []
}
```

`operation_state` may name more than one state at once, e.g. `OFF+STOP` for an
idle drive. Possible states are `RUN`, `STOP`, `ON`, `OFF`,
`TOOL_CHANGE_START`, `TOOL_CHANGE_END`, `WRITE_PROTECTION_DISABLE` and
`WRITE_PROTECTION_ENABLE`.

`alarm_flags` decodes `alarm_status` into names: `tool_overheated`,
`motor_drive_overheated`, `over_current`, `under_voltage`, `over_voltage`,
`self_test_running`, `rpm_drop`, `high_current`, `tool_change_in_progress`,
`tool_wiring_fault`, `factory_reset_mode`, `write_protection_disabled`.

Closing the component issues a best-effort STOP, so a crash or reconfigure does
not leave the spindle spinning.
