"""
Check whether a BCI2000 .dat file stores joystick X/Y coordinates.

By default, uses the first .dat file found in this script's directory.
Optionally pass a path:  python check_joystick_coords.py path/to/file.dat
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from BCI2000Tools.FileReader import bcistream


JOYSTICK_STATE_CANDIDATES = {
    "x": ("JoystickX", "JoyX", "JoystickPosX", "AnalogX", "CursorX"),
    "y": ("JoystickY", "JoyY", "JoystickPosY", "AnalogY", "CursorY"),
}


def find_default_dat(folder: Path) -> Path:
    matches = sorted(folder.glob("*.dat"))
    if not matches:
        raise FileNotFoundError(f"No .dat file found in {folder}")
    return matches[0]


def summarize_state(arr: np.ndarray) -> dict:
    a = np.asarray(arr).ravel()
    return {
        "min": float(a.min()),
        "max": float(a.max()),
        "mean": float(a.mean()),
        "n_unique": int(np.unique(a).size),
        "n_samples": int(a.size),
        "changes": int(np.count_nonzero(np.diff(a))),
    }


def find_axis_states(state_names: set[str]) -> dict[str, str | None]:
    found = {}
    for axis, candidates in JOYSTICK_STATE_CANDIDATES.items():
        found[axis] = next((name for name in candidates if name in state_names), None)
    return found


def check_joystick_coords(dat_path: Path) -> dict:
    stream = bcistream(str(dat_path))
    state_names = set(stream.statedefs.keys())
    params = stream.params

    axis_states = find_axis_states(state_names)
    has_x = axis_states["x"] is not None
    has_y = axis_states["y"] is not None
    stores_joystick_xy = has_x and has_y

    # Decode only the states we care about (plus a few useful task markers).
    wanted = [name for name in axis_states.values() if name]
    for extra in ("JoystickButton", "TaskPhase", "TargetIndex", "CursorX", "CursorY"):
        if extra in state_names and extra not in wanted:
            wanted.append(extra)

    _, states = stream.decode(nsamp="all", states=wanted or "all")
    summaries = {name: summarize_state(states[name]) for name in wanted}

    app_chain = params.get("ApplicationFilterChain", [])
    app_modules = [row[0] for row in app_chain] if app_chain else []

    return {
        "file": str(dat_path.resolve()),
        "n_channels": int(stream.nchan),
        "n_samples": int(stream.samples()),
        "sampling_rate_hz": float(stream.samplingfreq_hz),
        "application_modules": app_modules,
        "serial_port": params.get("SerialPort"),
        "all_state_names": sorted(state_names),
        "joystick_x_state": axis_states["x"],
        "joystick_y_state": axis_states["y"],
        "stores_joystick_xy": stores_joystick_xy,
        "state_summaries": summaries,
    }


def print_report(result: dict) -> None:
    print(f"File: {result['file']}")
    print(
        f"Recording: {result['n_channels']} channels, "
        f"{result['n_samples']} samples @ {result['sampling_rate_hz']} Hz"
    )
    if result["application_modules"]:
        print(f"Application modules: {', '.join(result['application_modules'])}")
    if result["serial_port"]:
        print(f"SerialPort param: {result['serial_port']}")

    print()
    if result["stores_joystick_xy"]:
        print("RESULT: YES - this file stores joystick X and Y coordinates.")
        print(f"  X state: {result['joystick_x_state']}")
        print(f"  Y state: {result['joystick_y_state']}")
    elif result["joystick_x_state"] or result["joystick_y_state"]:
        print("RESULT: PARTIAL - only one joystick axis state was found.")
        print(f"  X state: {result['joystick_x_state']}")
        print(f"  Y state: {result['joystick_y_state']}")
    else:
        print("RESULT: NO - no JoystickX / JoystickY (or known aliases) found in states.")

    if result["state_summaries"]:
        print("\nAxis / related state summaries:")
        for name, s in result["state_summaries"].items():
            print(
                f"  {name}: min={s['min']:.4g} max={s['max']:.4g} "
                f"mean={s['mean']:.4g} unique={s['n_unique']} changes={s['changes']}"
            )

    joy_like = [n for n in result["all_state_names"] if "joy" in n.lower() or "cursor" in n.lower()]
    if joy_like:
        print(f"\nJoystick/cursor-related states present: {', '.join(joy_like)}")


def main(argv: list[str]) -> int:
    if len(argv) > 1:
        dat_path = Path(argv[1])
    else:
        dat_path = find_default_dat(Path(__file__).resolve().parent)

    if not dat_path.is_file():
        print(f"Error: file not found: {dat_path}", file=sys.stderr)
        return 1

    result = check_joystick_coords(dat_path)
    print_report(result)
    return 0 if result["stores_joystick_xy"] else 2


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
