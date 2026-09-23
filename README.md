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

## Model viam:mirka:autochanger-remover

A **gantry**. Moves the Mirka AutoChanger remover knife anywhere along its
25 mm travel, and publishes a kinematic model so motion planning sees the blade
where it actually is. The knife is a Festo
EGSS-BS slide driven over IO-Link through an IFM AL1342 master, which the module
reaches with ifm's IoT Core JSON API over HTTP.

Page references below are to *Manual - Mirka AutoChanger*, downloadable from
[Mirka's technical documents](https://www.mirka.com/en-us/support/downloads/technical-documents/?currentPage=1&searchTerm=autochanger).

A gantry rather than a generic component because a frame system link carries one
geometry and this needs three — body, blade, and the head that travels with the
blade. Only a kinematic model holds a geometry per link.

Motion goes through the gantry methods; `DoCommand` carries only what they
cannot express.

This model does **not** sequence the disc-removal cycle — that interleaves knife
positions with robot waypoints, so it belongs to an orchestration service
holding both.

Homing is deliberately not a command. See [Commissioning](#commissioning-homing).

### Kinematic model

```
body          static                geometry 195 x 250 x 450
knife_joint   prismatic, +X, 0-25 mm
blade         rides the joint       geometry   2 x 180 x 260
head          rides the blade       geometry  20 x 180 x 120
```

Origin is the sliding plate's front face, centred in Y, at the top face the
body and blade share. `+X` is the direction the knife extends, `+Y` across the
plate, `+Z` up, so every geometry hangs below `z=0`. Units are millimetres.

Each axis is anchored to something that can be physically touched, which is
what makes the mounting frame calibratable by touching off with the arm.

### Gantry methods

| Method | Behaviour |
|---|---|
| `Kinematics` | the model above |
| `CurrentInputs` | the knife's current offset along its travel, in mm |
| `GoToInputs` | moves to any offset on the axis |
| `Position` | the same offset, in mm |
| `MoveToPosition` | moves to any offset on the axis; speeds are ignored |
| `Lengths` | `[25]` |
| `IsMoving` | the drive's motion bit |
| `Stop` | halts motion |
| `Home` | **refuses** — see [Commissioning](#commissioning-homing) |

The axis is continuous over `0` to `25` mm. Offsets off it are refused rather
than snapped, so a caller asking for 40 mm is told, not silently landed at the
end stop.

The three named knife positions are the drive's command vocabulary, not its
reachable set. The ends route to the mechanical end stops; everything between is
reached by writing the intermediate target. Arrival at an end stop is decided by
its process-data bit, anywhere else by comparing the position register against
the target — every intermediate point reports the same bit.

A background poller owns the connection and refreshes a snapshot every 100 ms;
every read method returns a field of it and performs no I/O. The frame system
resolves this component once per node it evaluates, so serving each caller from
the device meant dozens of simultaneous requests for one question.

Position comes from the drive's actual-position register (ISDU `0x0120`), not
the process-data bits, which assert only at the three taught points.

The poller reads process data every cycle (~25 ms, a cyclic value the master
already holds) and the position register only when a move settles, because an
acyclic read costs ~860 ms. Nothing consumes the knife's position mid-stroke —
the arm is stationary whenever it is near the changer.

### Configuration for autochanger-remover model

The following attribute template can be used to configure this model:

```json
{
    "address": <string>,
    "port": <int>,
    "timeout_ms": <uint>
}
```

#### Attributes

The following attributes are available for this model:

| Name          | Type   | Inclusion | Description                                                                                             | Default |
| ------------- | ------ | --------- | --------------------------------------------------------------------------------------------------------- | ------- |
| `address`     | string | Required  | AL1342 **IoT** interface, host or `host:port`. Not the fieldbus interface — see [Networking](#networking). |         |
| `port`        | int    | Optional  | IO-Link port the remover occupies, 1–8.                                                                    | 1       |
| `timeout_ms`  | uint   | Optional  | How long a commanded move may take to reach its position.                                                  | 10000   |

`timeout_ms` bounds *physical* completion, not the network: a jam answers every
HTTP request perfectly while never reaching its target, so the request timeout
(a constant, 3 s) cannot catch it.

There is no geometry configuration. The envelope is a property of the hardware
and lives in `autochanger/model.json`.

#### Knife positions

The manual (p.49) names three positions and the drive has a command for each.
Not the reachable set — the axis is continuous. `GetStatus` reports the drive's
own state names, because `intermediate` asserts for every point between the end
stops and cannot say which.

| Manual  | Drive reports  | Offset | Description                                                       |
| ------- | -------------- | ------ | ----------------------------------------------------------------- |
| 2       | `in`           | `0`    | Knife flush against the sliding plate.                            |
| 1       | `intermediate` | tuned  | Extended just enough to admit an abrasive. See [Tuning the grip position](#tuning-the-grip-position). |
| 3       | `out`          | `25`   | Fully out, releasing the disc.                                    |

#### Example Configuration

```json
{
    "address": "192.168.50.10"
}
```

The component's `api` is `rdk:component:gantry`:

```json
{
    "name": "remover",
    "api": "rdk:component:gantry",
    "model": "viam:mirka:autochanger-remover",
    "attributes": { "address": "192.168.50.10" },
    "frame": {
        "parent": "world",
        "translation": { "x": 0, "y": 0, "z": 0 },
        "orientation": { "type": "ov_degrees", "value": { "x": 0, "y": 0, "z": 1, "th": 0 } }
    }
}
```

### DoCommand

#### quit_error DoCommand

Acknowledges a latched drive fault. The drive drops `ready` and refuses every
command until acknowledged. Edge triggered, so this asserts then clears the bit.

```json
{ "command": "quit_error" }
```

Response:

```json
{ "acknowledged": true }
```

#### diagnostics DoCommand

Transport and drive state, for telling whether a fault is the link, the master
or the drive. Reads the device live.

```json
{ "command": "diagnostics" }
```

Response (abridged):

```json
{
    "module_age": "1m33s",
    "live_read_ok": true,
    "position": "intermediate",
    "position_counts": 2175,
    "position_mm": 19.99,
    "last_offset_mm": 19.99,
    "transport": {
        "requests": 665,
        "errors": 0,
        "by_class": { "ok": 665 },
        "by_adr": {
            "pdin/getdata":   { "count": 639, "mean_rtt_ms": 6.9 },
            "iolreadacyclic": { "count": 16,  "mean_rtt_ms": 861.7 }
        },
        "max_in_flight": 2,
        "max_wait_ms": 1060.9,
        "first_ok_after": "2ms"
    }
}
```

`by_class` counts failures by cause: `conn_reset` and `conn_refused` implicate
the master, `canceled` only means a caller stopped listening. `position_mm` is a
live read, `last_offset_mm` what the poller last published — they diverge only if
the poller has stopped tracking.

### Status

State is served through `GetStatus`. There is no `status` DoCommand for this
model.

```json
{
    "position": "intermediate",
    "moving": false,
    "ready": true,
    "raw": 24
}
```

| Field         | Description                                                                 |
| ------------- | --------------------------------------------------------------------------- |
| `position`    | `in`, `out`, `intermediate`, or `unknown`.                                   |
| `moving`      | The drive reports itself in motion.                                          |
| `ready`       | The drive is ready to accept commands.                                       |
| `raw`         | The process-data word, for diagnosis.                                        |

`position` is decoded from the drive, never inferred from what was last
commanded, so it is correct immediately after a restart. `unknown` means no
position bit is set — in motion, or parked where none of the three applies. The
reported geometry is exact either way; it comes from the position register.

### Frame configuration

Configure the component's `frame` with `parent`, `translation` and `orientation`,
and **without** its optional `geometry` field.

That field replaces the whole kinematic envelope with one static box, silently
discarding the three this model publishes.

The model is expressed in the component's own frame, so mounting lives entirely
in `frame`.

Three geometries reach the planner, one per link:

| Link    | Description                                          |
| ------- | ---------------------------------------------------- |
| `body`  | Frame, motor and sliding plate. Static.              |
| `blade` | Rides the joint; tracks the current position.        |
| `head`  | The Mirka head, carried by the blade.                |

Separate so a caller can permit the collisions the removal process requires —
the sander presses against the sliding plate — without permitting contact with
the blade.

### Commissioning: homing

Required once per unit, fresh from the box. Deliberately **not** a module
command: the precondition is physical and Viam cannot enforce it.

1. **Remove the sliding plate from the Remover** (AutoChanger manual p.49).
2. Clear the knife's full travel path in both directions.
3. Send the reference command once:

```bash
curl -s -X POST http://<address> -H 'Content-Type: application/json' \
  -d '{"code":"request","cid":1,"adr":"/iolinkmaster/port[1]/iolinkdevice/iolwriteacyclic","data":{"index":2,"subindex":0,"value":"CE"}}'
```

**The value is `CE`.** `CD` is Restore Factory Settings.

4. Watch until the drive settles; a two-pass stroke detection is normal:

```bash
curl -s -X POST http://<address> -H 'Content-Type: application/json' \
  -d '{"code":"request","cid":1,"adr":"/iolinkmaster/port[1]/iolinkdevice/pdin/getdata"}'
```

5. Refit the sliding plate.

Homing with the plate fitted teaches a zero short by the plate's thickness. The
symptom appears much later, as discs failing to release (manual p.55).

Homing also overwrites the intermediate position parameter with the full stroke.
The module rewrites it before every intermediate move, so re-homing is safe.

### Tuning the grip position

The grip position is where the knife is open just enough to admit an abrasive.
It is a **process parameter, not a hardware fact** — it depends on the disc, so
the caller holds it, not this module.

Tune it with `MoveToPosition` until a disc slides in with slight clearance and no
force, approaching from flush each time. Roughly 0.7 mm suits a standard
abrasive.

To measure the slot instead, measure from the **plate's front face to the
knife's rear face** — the surface the abrasive bears against. Measuring to the
front of the blade includes its thickness and gives roughly twice the real gap.
That is a *gap*: subtract 0.42 mm, the clearance at the drive's zero, for the
joint offset.


### Known limitations

- **The air nozzle is not installed.** The pneumatic kit blows the released disc
  clear at position 3. Without it a removal may peel a disc without clearing it.
- **The body is wider than the blade.** 250 mm against 180, so the body sets the
  approach clearance.
- **The model puts the blade 0.42 mm too far in.** The geometry places its rear
  face on the plate at joint zero, where the drive's zero leaves that much
  clearance. Constant across the stroke, and understates the blade's reach.
- **`failsafeiolink` is unresolved.** The master's fail-safe output setting for
  this port reads `0` and that enum's meaning is unestablished. It decides what
  the outputs do when the controller disconnects.

## Networking

The AL1342 has **two independent network interfaces**, with different MAC
addresses and separate IP configuration:

| Interface | Serves                                          | Ships as                            |
| --------- | ----------------------------------------------- | ----------------------------------- |
| IoT       | HTTP / IoT Core JSON — what this module uses     | DHCP, falling back to link-local    |
| Fieldbus  | Modbus TCP                                       | static, `192.168.1.250`             |

They are not bridged and sit on different physical ports. `address` must be the
**IoT** interface.

Give it a static address. A link-local address is self-assigned and not stable
across conflicts or resets, so a machine config naming one works until the day it
silently does not.

The host NIC facing it needs a persistent address too — `ip addr add` does not
survive a reboot, and viam-server would then fail to reach the master with only a
connect timeout to explain it. Configure it with **no gateway**, or a
point-to-point industrial link will displace the default route on the interface
carrying general traffic.
