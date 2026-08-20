#!/usr/bin/env python3
"""
PAT 3-channel calibration logger.

Expected STM32 UART line:
    CAL,<A1_mV_RMS>,<P1_mdeg>,<A2_mV_RMS>,<P2_mdeg>,<A3_mV_RMS>,<P3_mdeg>

Example:
    CAL,580,-135021,218,46334,76,77590

Features:
- Auto-detects ST-LINK virtual COM port on Linux, or accepts --port.
- Logs every valid measurement to CSV.
- Displays rolling amplitude for CH1-CH3.
- Computes rolling circular mean phase and circular phase jitter.
- Optional live matplotlib plots.

Dependencies:
    pip install pyserial matplotlib
"""

from __future__ import annotations

import argparse
import csv
import glob
import math
import sys
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

try:
    import serial
except ImportError:
    print("Missing dependency: pyserial")
    print("Install with: pip install pyserial")
    raise

try:
    import matplotlib.pyplot as plt
except ImportError:
    plt = None


CHANNELS = 3


def wrap_deg(angle_deg: float) -> float:
    """Wrap angle to [-180, 180)."""
    return (angle_deg + 180.0) % 360.0 - 180.0


def circular_mean_deg(values_deg) -> float:
    """Circular mean in degrees, returned in [-180, 180)."""
    if not values_deg:
        return float("nan")

    s = sum(math.sin(math.radians(v)) for v in values_deg)
    c = sum(math.cos(math.radians(v)) for v in values_deg)

    if abs(s) < 1e-15 and abs(c) < 1e-15:
        return float("nan")

    return wrap_deg(math.degrees(math.atan2(s, c)))


def circular_std_deg(values_deg) -> float:
    """
    Circular standard deviation in degrees.

    Uses:
        R = sqrt(mean(cos)^2 + mean(sin)^2)
        sigma = sqrt(-2 ln R)
    """
    n = len(values_deg)
    if n == 0:
        return float("nan")

    mean_s = sum(math.sin(math.radians(v)) for v in values_deg) / n
    mean_c = sum(math.cos(math.radians(v)) for v in values_deg) / n
    r = math.hypot(mean_s, mean_c)

    # Numerical protection.
    r = min(max(r, 1e-12), 1.0)
    return math.degrees(math.sqrt(-2.0 * math.log(r)))


def find_stlink_port() -> str:
    candidates = sorted(
        glob.glob("/dev/serial/by-id/*STMicroelectronics*STLINK*")
        + glob.glob("/dev/serial/by-id/*STLink*")
        + glob.glob("/dev/serial/by-id/*STLINK*")
    )

    if not candidates:
        raise RuntimeError(
            "Could not auto-detect an ST-LINK serial port. "
            "Use --port /dev/ttyACM0 or the full /dev/serial/by-id/... path."
        )

    return candidates[0]


def parse_cal_line(line: str):
    """
    Parse:
        CAL,A1,P1_mdeg,A2,P2_mdeg,A3,P3_mdeg

    Returns:
        amplitudes_mV: [a1, a2, a3]
        phases_deg:    [p1, p2, p3]
    """
    parts = line.strip().split(",")

    if len(parts) != 7 or parts[0] != "CAL":
        return None

    try:
        a1 = int(parts[1])
        p1 = int(parts[2]) / 1000.0
        a2 = int(parts[3])
        p2 = int(parts[4]) / 1000.0
        a3 = int(parts[5])
        p3 = int(parts[6]) / 1000.0
    except ValueError:
        return None

    return [a1, a2, a3], [wrap_deg(p1), wrap_deg(p2), wrap_deg(p3)]


def make_default_csv_name() -> str:
    log_dir = Path("sample_log3")
    log_dir.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return str(log_dir / f"pat_calibration_{stamp}.csv")


def print_status(amplitude_history, phase_history, sample_count: int):
    print("\033[2J\033[H", end="")  # clear terminal, cursor home
    print(f"PAT 3-channel calibration logger | samples: {sample_count}")
    print("Rolling statistics use only the current window.\n")

    for ch in range(CHANNELS):
        amps = amplitude_history[ch]
        phases = phase_history[ch]

        latest_amp = amps[-1] if amps else float("nan")
        latest_phase = phases[-1] if phases else float("nan")
        mean_amp = sum(amps) / len(amps) if amps else float("nan")
        mean_phase = circular_mean_deg(phases)
        jitter = circular_std_deg(phases)

        print(
            f"CH{ch+1}: "
            f"A={latest_amp:7.1f} mV RMS  "
            f"Amean={mean_amp:7.1f} mV  "
            f"P={latest_phase:8.3f} deg  "
            f"Pmean={mean_phase:8.3f} deg  "
            f"jitter={jitter:6.3f} deg"
        )

    sys.stdout.flush()


def setup_plot(history_len: int):
    if plt is None:
        return None

    plt.ion()

    fig_amp, ax_amp = plt.subplots()
    amp_lines = []
    for ch in range(CHANNELS):
        line, = ax_amp.plot([], [], label=f"CH{ch+1}")
        amp_lines.append(line)
    ax_amp.set_title("PAT receiver amplitude")
    ax_amp.set_xlabel("Sample")
    ax_amp.set_ylabel("Amplitude (mV RMS)")
    ax_amp.set_ylim(0, 700)
    ax_amp.legend()
    ax_amp.grid(True)

    fig_phase, ax_phase = plt.subplots()
    phase_lines = []
    for ch in range(CHANNELS):
        line, = ax_phase.plot([], [], label=f"CH{ch+1}")
        phase_lines.append(line)
    ax_phase.set_title("PAT receiver phase")
    ax_phase.set_xlabel("Sample")
    ax_phase.set_ylabel("Phase (deg)")
    ax_phase.set_ylim(-180, 180)
    ax_phase.legend()
    ax_phase.grid(True)

    return {
        "fig_amp": fig_amp,
        "ax_amp": ax_amp,
        "amp_lines": amp_lines,
        "fig_phase": fig_phase,
        "ax_phase": ax_phase,
        "phase_lines": phase_lines,
        "history_len": history_len,
    }


def update_plot(plot_state, amplitude_history, phase_history):
    if plot_state is None:
        return

    n = max(len(amplitude_history[0]), 1)
    x = list(range(n))


    for ch in range(CHANNELS):
        amps = list(amplitude_history[ch])
        phases = list(phase_history[ch])

        plot_state["amp_lines"][ch].set_data(x[-len(amps):], amps)
        plot_state["phase_lines"][ch].set_data(x[-len(phases):], phases)


    plot_state["ax_amp"].set_xlim(0, max(n - 1, 1))

    plot_state["ax_phase"].set_xlim(0, max(n - 1, 1))

    plot_state["fig_amp"].canvas.draw_idle()
    plot_state["fig_phase"].canvas.draw_idle()
    plt.pause(0.001)


def main():
    parser = argparse.ArgumentParser(
        description="Read 3-channel PAT calibration data from STM32 UART."
    )
    parser.add_argument(
        "--port",
        default=None,
        help="Serial port. If omitted, auto-detect an ST-LINK /dev/serial/by-id port.",
    )
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--window",
        type=int,
        default=50,
        help="Rolling window length for mean/jitter statistics (default: 50 samples).",
    )
    parser.add_argument(
        "--csv",
        default=None,
        help="CSV output path. Default: pat_calibration_YYYYMMDD_HHMMSS.csv",
    )
    parser.add_argument(
        "--no-plot",
        action="store_true",
        help="Disable matplotlib live plots and show terminal statistics only.",
    )
    parser.add_argument(
        "--refresh",
        type=float,
        default=0.5,
        help="Terminal/plot refresh interval in seconds (default: 0.5).",
    )
    args = parser.parse_args()

    if args.window < 2:
        raise SystemExit("--window must be >= 2")

    port = args.port or find_stlink_port()
    csv_path = Path(args.csv or make_default_csv_name())

    amplitude_history = [
        deque(maxlen=args.window) for _ in range(CHANNELS)
    ]
    phase_history = [
        deque(maxlen=args.window) for _ in range(CHANNELS)
    ]

    plot_state = None
    if not args.no_plot:
        if plt is None:
            print("matplotlib is not installed; continuing without plots.")
        else:
            plot_state = setup_plot(args.window)

    print(f"Serial port : {port}")
    print(f"Baud rate   : {args.baud}")
    print(f"CSV log     : {csv_path}")
    print(f"Window      : {args.window} samples")
    print("Press Ctrl+C to stop.\n")

    start_monotonic = time.monotonic()
    sample_count = 0
    last_refresh = 0.0

    fieldnames = [
        "timestamp_iso",
        "elapsed_s",
        "sample",
        "ch1_amplitude_mV_RMS",
        "ch1_phase_deg",
        "ch2_amplitude_mV_RMS",
        "ch2_phase_deg",
        "ch3_amplitude_mV_RMS",
        "ch3_phase_deg",
    ]

    try:
        with serial.Serial(port, args.baud, timeout=1.0) as ser, \
             csv_path.open("w", newline="") as f:

            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            f.flush()

            # Give the virtual COM port a moment to settle.
            time.sleep(0.2)
            ser.reset_input_buffer()

            while True:
                raw = ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="replace").strip()
                parsed = parse_cal_line(line)
                if parsed is None:
                    continue

                amplitudes, phases = parsed
                sample_count += 1

                for ch in range(CHANNELS):
                    amplitude_history[ch].append(float(amplitudes[ch]))
                    phase_history[ch].append(float(phases[ch]))

                now = datetime.now(timezone.utc)
                elapsed = time.monotonic() - start_monotonic

                writer.writerow({
                    "timestamp_iso": now.isoformat(),
                    "elapsed_s": f"{elapsed:.6f}",
                    "sample": sample_count,
                    "ch1_amplitude_mV_RMS": amplitudes[0],
                    "ch1_phase_deg": f"{phases[0]:.3f}",
                    "ch2_amplitude_mV_RMS": amplitudes[1],
                    "ch2_phase_deg": f"{phases[1]:.3f}",
                    "ch3_amplitude_mV_RMS": amplitudes[2],
                    "ch3_phase_deg": f"{phases[2]:.3f}",
                })
                f.flush()

                now_monotonic = time.monotonic()
                if (now_monotonic - last_refresh) >= args.refresh:
                    last_refresh = now_monotonic
                    print_status(amplitude_history, phase_history, sample_count)
                    update_plot(plot_state, amplitude_history, phase_history)

    except KeyboardInterrupt:
        print("\nStopped by user.")
        print(f"Saved {sample_count} samples to: {csv_path}")
    except serial.SerialException as exc:
        print(f"\nSerial error: {exc}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
