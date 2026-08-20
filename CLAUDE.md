# viam-mirka

Viam module for Mirka robotic sanding hardware. Go only — no Python anywhere in this repo.

## Layout

- `airos/airos.go` — the single model `viam:mirka:airos-550cv`: a `generic` component driving the Mirka AIROS 550CV orbital sander through its motor drive cabinet over **Modbus RTU** (serial, `github.com/simonvetter/modbus`).
- `cmd/module/cmd.go` — module entrypoint (`module.ModularMain`).
- `examples/test_sander/main.go` — CLI tester that connects to a live machine via the Viam Go SDK and exercises DoCommand (`status`, `start`, `stop`, `set-speed`, `monitor`, `bench-test`). Stdlib `flag` only.
- `meta.json` — module id `viam:mirka`, entrypoint `./viam-mirka`, built for linux/{arm64,amd64} + darwin/arm64.

## Build / test

- `make module` — cross-compiles for linux/arm64 (the deploy target) and tars it. Plain `go build ./...` for local checks.
- `make test`, `make lint` (gofmt -s).

## Hardware / protocol notes

- Register constants in `airos.go` are 0-indexed wire addresses; trailing comments give the 1-indexed block-prefixed numbers (40011 etc.) used by the Mirka manual. Keep both in sync when adding registers.
- Firmware 3.05+ dropped ON/OFF (0x0004/0x0008) and WP writes from the Operation register — ON/OFF is the DI1 hardware line. Don't reintroduce them; the drive answers Modbus exception 0x04.
- Speed setpoint is clamped to 4000–10000 RPM in code because the drive silently misbehaves outside that band.
- `Close()` best-effort STOPs the spindle so a crash/reconfigure never leaves it spinning. Preserve that invariant in any new motion-capable component.

## Conventions

- Components are `generic` API models controlled through `DoCommand` with a `{"command": ...}` envelope; errors name the offending field and the supported command list.
- Config defaults are applied in the constructor, not `Validate` (Validate only rejects impossible values).
