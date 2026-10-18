"""Stream robot_state.json poses to a five-servo Arduino controller."""

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

from robot_contract import JOINT_LIMITS


JOINT_RANGES = {
    **JOINT_LIMITS,
    "gripper": (0.0, 100.0),
}
JOINTS = ("base", "joint1", "joint2", "joint3", "gripper")
DEFAULT_STATE_FILE = Path(__file__).resolve().parent / "robot_state.json"


def load_servo_positions(path: Path) -> tuple[int, ...]:
    """Read and validate the pose, mapping each joint's full range to 0..180."""
    with path.open(encoding="utf-8") as state_file:
        state: Any = json.load(state_file)

    if not isinstance(state, dict):
        raise ValueError(f"{path} must contain a JSON object.")

    positions = []
    for joint in JOINTS:
        value = state.get(joint)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{path}: {joint} must be a number.")
        minimum, maximum = JOINT_RANGES[joint]
        if not minimum <= value <= maximum:
            raise ValueError(
                f"{path}: {joint}={value} is outside {minimum}..{maximum}."
            )
        positions.append(round((value - minimum) * 180 / (maximum - minimum)))
    return tuple(positions)


def state_signature(path: Path) -> tuple[int, int]:
    stat = path.stat()
    return stat.st_mtime_ns, stat.st_size


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Watch robot_state.json and send five servo positions over serial."
    )
    parser.add_argument("--port", required=True, help="Arduino serial port (for example COM3)")
    parser.add_argument("--state", type=Path, default=DEFAULT_STATE_FILE)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--poll", type=float, default=0.1, help="File polling interval in seconds")
    args = parser.parse_args()
    if args.poll <= 0:
        parser.error("--poll must be greater than zero")

    try:
        import serial
    except ImportError:
        print("pyserial is required; install it with: pip install pyserial", file=sys.stderr)
        return 1

    try:
        with serial.Serial(args.port, args.baud, timeout=1, write_timeout=1) as connection:
            # Opening a serial port resets most Arduino boards.
            time.sleep(2)
            last_signature = None
            print(f"Watching {args.state} on {args.port} at {args.baud} baud.")
            while True:
                signature = state_signature(args.state)
                if signature != last_signature:
                    positions = load_servo_positions(args.state)
                    connection.write((",".join(map(str, positions)) + "\n").encode("ascii"))
                    connection.flush()
                    response = connection.readline().decode("ascii", errors="replace").strip()
                    if response != "OK":
                        raise RuntimeError(f"Arduino rejected pose (response: {response!r}).")
                    last_signature = signature
                    print(f"Sent servo positions: {', '.join(map(str, positions))}")
                time.sleep(args.poll)
    except KeyboardInterrupt:
        print("\nStopped.")
    except (OSError, ValueError, RuntimeError) as error:
        print(f"Servo controller error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
