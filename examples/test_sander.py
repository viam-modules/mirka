#!/usr/bin/env python3
# Usage:
#   pip install viam-sdk
#   export VIAM_ADDRESS=...        # e.g. my-machine-main.xxxx.viam.cloud
#   export VIAM_API_KEY_ID=...
#   export VIAM_API_KEY=...
#   python test_sander.py status
#   python test_sander.py start --rpm 4000
#   python test_sander.py stop
#   python test_sander.py set-speed 6000
#   python test_sander.py monitor --interval 1
#   python test_sander.py bench-test

import argparse
import asyncio
import os
import sys
from datetime import datetime

from viam.components.generic import Generic
from viam.robot.client import RobotClient


def env(name: str) -> str:
    value = os.environ.get(name)
    if not value:
        sys.exit(f"missing env var {name}")
    return value


async def connect() -> RobotClient:
    opts = RobotClient.Options.with_api_key(
        api_key=env("VIAM_API_KEY"),
        api_key_id=env("VIAM_API_KEY_ID"),
    )
    return await RobotClient.at_address(env("VIAM_ADDRESS"), opts)


def fmt_status(s: dict) -> str:
    flags = s.get("alarm_flags") or []
    flags_str = ", ".join(flags) if flags else "—"
    return (
        f"state={s.get('operation_state'):<7} "
        f"running={s.get('running')!s:<5} "
        f"setpoint={s.get('speed_setpoint_rpm')} rpm  "
        f"actual={s.get('average_speed_rpm')} rpm  "
        f"tool={s.get('tool_temp_c')}°C  drive={s.get('drive_temp_c')}°C  "
        f"alarms=[{flags_str}]"
    )


async def cmd_status(sander):
    result = await sander.do_command({"command": "status"})
    print(fmt_status(result))


async def cmd_start(sander, rpm: int):
    result = await sander.do_command({"command": "start", "rpm": rpm})
    print(result)


async def cmd_stop(sander):
    result = await sander.do_command({"command": "stop"})
    print(result)


async def cmd_set_speed(sander, rpm: int):
    result = await sander.do_command({"command": "set_speed", "rpm": rpm})
    print(result)


async def cmd_monitor(sander, interval: float):
    print(f"polling every {interval}s — Ctrl-C to stop")
    try:
        while True:
            result = await sander.do_command({"command": "status"})
            print(f"{datetime.now():%H:%M:%S}  {fmt_status(result)}")
            await asyncio.sleep(interval)
    except KeyboardInterrupt:
        print("stopped")


# Canonical first-spin verification: clamped sander, no disc, low RPM, short run.
# Prompts before any motion to avoid surprises.
async def cmd_bench_test(sander):
    print("BENCH TEST — pre-flight checks:")
    print("  1. Sander is clamped to a fixture and cannot move")
    print("  2. Sanding disc is REMOVED")
    print("  3. No body parts or loose items within arm's reach")
    print("  4. Cabinet E-stop reachable")
    resp = input("All four confirmed? (type YES to continue): ")
    if resp.strip() != "YES":
        print("aborted")
        return

    print("\n[1/4] status before start")
    print("  " + fmt_status(await sander.do_command({"command": "status"})))

    print("\n[2/4] start at 4000 rpm")
    await sander.do_command({"command": "start", "rpm": 4000})
    await asyncio.sleep(0.5)
    print("  " + fmt_status(await sander.do_command({"command": "status"})))

    print("\n[3/4] hold 2s")
    await asyncio.sleep(2.0)
    print("  " + fmt_status(await sander.do_command({"command": "status"})))

    print("\n[4/4] stop")
    await sander.do_command({"command": "stop"})
    await asyncio.sleep(0.5)
    print("  " + fmt_status(await sander.do_command({"command": "status"})))

    print("\ndone")


async def main():
    parser = argparse.ArgumentParser(description="CLI tester for viam:mirka:airos-550cv")
    parser.add_argument(
        "--component",
        default=os.environ.get("MIRKA_COMPONENT", "mirka-sander"),
        help="resource name of the generic component (default: mirka-sander or $MIRKA_COMPONENT)",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("status")

    start = sub.add_parser("start")
    start.add_argument("--rpm", type=int, default=4000)

    sub.add_parser("stop")

    set_speed = sub.add_parser("set-speed")
    set_speed.add_argument("rpm", type=int)

    monitor = sub.add_parser("monitor")
    monitor.add_argument("--interval", type=float, default=1.0)

    sub.add_parser("bench-test")

    args = parser.parse_args()

    machine = await connect()
    try:
        sander = Generic.from_robot(machine, args.component)

        if args.command == "status":
            await cmd_status(sander)
        elif args.command == "start":
            await cmd_start(sander, args.rpm)
        elif args.command == "stop":
            await cmd_stop(sander)
        elif args.command == "set-speed":
            await cmd_set_speed(sander, args.rpm)
        elif args.command == "monitor":
            await cmd_monitor(sander, args.interval)
        elif args.command == "bench-test":
            await cmd_bench_test(sander)
    finally:
        await machine.close()


if __name__ == "__main__":
    asyncio.run(main())
