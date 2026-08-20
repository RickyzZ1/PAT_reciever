#!/usr/bin/env python3
"""
PAT 3-RX phase triangulation / calibration tool.

Coordinate convention
---------------------
PAT: 16 x 16 transducers, 10 mm pitch, z = 0.

TX numbering increases along +y first:
    TX0  = (0,   0, 0)
    TX15 = (0, 150, 0)
    TX16 = (10,  0, 0)

So:
    x = (tx_id // 16) * 10 mm
    y = (tx_id %  16) * 10 mm

Nominal receiver geometry:
    RX1 = (  5, 135, 120) mm
    RX2 = (105, 135, 120) mm
    RX3 = (  5,  45, 120) mm

The STM32 phase is a delay phase relative to PAT_REF, so the free-space
model uses +k*d, where k = 360 / wavelength in deg/mm.

Why calibration is needed
-------------------------
At 40 kHz, 1 mm path error is about 42 degrees.  The integer receiver
coordinates are therefore only a nominal starting point.  This tool fits
a rigid receiver-board pose plus the two channel-pair phase offsets:

    dx, dy, z, yaw, O21, O31

using multiple known TX CSV files.

Each known TX contributes:
    Δφ21 = wrap(φ2 - φ1)
    Δφ31 = wrap(φ3 - φ1)

After calibration, "locate" ranks all 256 PAT transducer centers for an
unknown measurement CSV.

Dependencies:
    python3 -m pip install numpy scipy
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import random
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares


PAT_N = 16
PAT_PITCH_MM = 10.0
FREQ_HZ = 40_000.0

NOMINAL_RX = np.array(
    [
        [5.0, 135.0, 120.0],   # RX1
        [105.0, 135.0, 120.0], # RX2
        [5.0, 45.0, 120.0],    # RX3
    ],
    dtype=float,
)

RX_XY_CENTROID = NOMINAL_RX[:, :2].mean(axis=0)


@dataclass
class Measurement:
    tx_id: int | None
    path: str
    n: int
    amp_mean: np.ndarray
    d21_mean_deg: float
    d31_mean_deg: float
    d21_jitter_deg: float
    d31_jitter_deg: float


def wrap_deg(x):
    """Wrap degrees to [-180, 180)."""
    return (np.asarray(x) + 180.0) % 360.0 - 180.0


def circular_mean_deg(values) -> float:
    values = np.asarray(values, dtype=float)
    r = np.deg2rad(values)
    return float(np.rad2deg(np.arctan2(np.mean(np.sin(r)), np.mean(np.cos(r)))))


def circular_std_deg(values) -> float:
    values = np.asarray(values, dtype=float)
    r = np.deg2rad(values)
    mean_s = np.mean(np.sin(r))
    mean_c = np.mean(np.cos(r))
    R = float(np.hypot(mean_s, mean_c))
    R = min(max(R, 1e-12), 1.0)
    return math.degrees(math.sqrt(-2.0 * math.log(R)))


def tx_position(tx_id: int) -> np.ndarray:
    if not 0 <= tx_id < 256:
        raise ValueError(f"TX id must be 0..255, got {tx_id}")
    x = (tx_id // PAT_N) * PAT_PITCH_MM
    y = (tx_id % PAT_N) * PAT_PITCH_MM
    return np.array([x, y, 0.0], dtype=float)


def read_measurement(path: str, tx_id: int | None = None) -> Measurement:
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(path)

    a1, a2, a3 = [], [], []
    p1, p2, p3 = [], [], []

    with p.open("r", newline="") as f:
        reader = csv.DictReader(f)
        required = {
            "ch1_amplitude_mV_RMS",
            "ch1_phase_deg",
            "ch2_amplitude_mV_RMS",
            "ch2_phase_deg",
            "ch3_amplitude_mV_RMS",
            "ch3_phase_deg",
        }
        if not required.issubset(set(reader.fieldnames or [])):
            raise ValueError(f"{path}: missing expected STM32 logger columns")

        for row in reader:
            a1.append(float(row["ch1_amplitude_mV_RMS"]))
            a2.append(float(row["ch2_amplitude_mV_RMS"]))
            a3.append(float(row["ch3_amplitude_mV_RMS"]))
            p1.append(float(row["ch1_phase_deg"]))
            p2.append(float(row["ch2_phase_deg"]))
            p3.append(float(row["ch3_phase_deg"]))

    if len(p1) < 5:
        raise ValueError(f"{path}: not enough samples ({len(p1)})")

    p1 = np.asarray(p1)
    p2 = np.asarray(p2)
    p3 = np.asarray(p3)

    d21 = wrap_deg(p2 - p1)
    d31 = wrap_deg(p3 - p1)

    return Measurement(
        tx_id=tx_id,
        path=str(p),
        n=len(p1),
        amp_mean=np.array([np.mean(a1), np.mean(a2), np.mean(a3)]),
        d21_mean_deg=circular_mean_deg(d21),
        d31_mean_deg=circular_mean_deg(d31),
        d21_jitter_deg=circular_std_deg(d21),
        d31_jitter_deg=circular_std_deg(d31),
    )


def parse_sample_arg(text: str):
    """
    Parse:
        14=sample_log/file.csv
    """
    if "=" not in text:
        raise argparse.ArgumentTypeError("Use TXID=path, e.g. 14=sample_log/tx14.csv")
    left, right = text.split("=", 1)
    try:
        tx_id = int(left)
    except ValueError:
        raise argparse.ArgumentTypeError(f"Invalid TX id: {left}") from None
    if not 0 <= tx_id < 256:
        raise argparse.ArgumentTypeError("TX id must be 0..255")
    return tx_id, right


def transformed_receivers(params: np.ndarray) -> np.ndarray:
    """
    params = [dx, dy, z, yaw_deg, O21, O31]

    Rotate the nominal RX layout around its XY centroid, then translate it.
    """
    dx, dy, z_mm, yaw_deg, _, _ = params
    th = math.radians(yaw_deg)

    R = np.array(
        [
            [math.cos(th), -math.sin(th)],
            [math.sin(th),  math.cos(th)],
        ]
    )

    local = NOMINAL_RX[:, :2] - RX_XY_CENTROID
    xy = local @ R.T + RX_XY_CENTROID + np.array([dx, dy])

    return np.column_stack((xy, np.full(3, z_mm)))


def predicted_pair_phases(tx_id: int, params: np.ndarray, speed_m_s: float):
    rx = transformed_receivers(params)
    tx = tx_position(tx_id)
    d = np.linalg.norm(rx - tx, axis=1)

    # wavelength in mm
    wavelength_mm = speed_m_s * 1000.0 / FREQ_HZ
    deg_per_mm = 360.0 / wavelength_mm

    _, _, _, _, o21, o31 = params

    # STM32 reports delay phase, therefore +k*(distance difference).
    p21 = float(wrap_deg(deg_per_mm * (d[1] - d[0]) + o21))
    p31 = float(wrap_deg(deg_per_mm * (d[2] - d[0]) + o31))

    return p21, p31


def phase_error_deg(meas: Measurement, params: np.ndarray, speed_m_s: float):
    assert meas.tx_id is not None
    p21, p31 = predicted_pair_phases(meas.tx_id, params, speed_m_s)
    e21 = float(wrap_deg(meas.d21_mean_deg - p21))
    e31 = float(wrap_deg(meas.d31_mean_deg - p31))
    return e21, e31


def fit_calibration(
    measurements: list[Measurement],
    speed_m_s: float,
    starts: int,
    seed: int,
):
    if len(measurements) < 4:
        raise ValueError("Use at least 4 known TX files; 8-12 distributed points is preferred.")

    rng = random.Random(seed)

    # [dx, dy, z, yaw, O21, O31]
    lower = np.array([-30.0, -30.0, 90.0, -30.0, -180.0, -180.0])
    upper = np.array([ 30.0,  30.0,150.0,  30.0,  180.0,  180.0])

    def residual(params):
        out = []

        for m in measurements:
            e21, e31 = phase_error_deg(m, params, speed_m_s)

            # Circular residual represented continuously by sin(error/2).
            # Weight noisy channels less strongly.
            s21 = max(m.d21_jitter_deg, 3.0)
            s31 = max(m.d31_jitter_deg, 3.0)

            scale21 = max(math.sin(math.radians(s21) / 2.0), 1e-4)
            scale31 = max(math.sin(math.radians(s31) / 2.0), 1e-4)

            out.append(math.sin(math.radians(e21) / 2.0) / scale21)
            out.append(math.sin(math.radians(e31) / 2.0) / scale31)

        # Very weak pose prior.  It only discourages remote wavelength aliases
        # when data are sparse; with many calibration points the data dominate.
        dx, dy, z_mm, yaw_deg, _, _ = params
        prior_strength = 0.05
        out.extend(
            [
                prior_strength * dx / 20.0,
                prior_strength * dy / 20.0,
                prior_strength * (z_mm - 120.0) / 20.0,
                prior_strength * yaw_deg / 15.0,
            ]
        )

        return np.asarray(out)

    starts = max(starts, 1)
    best = None

    initial_guesses = [
        np.array([0.0, 0.0, 120.0, 0.0, 0.0, 0.0])
    ]

    for _ in range(starts - 1):
        initial_guesses.append(
            np.array(
                [
                    rng.uniform(-25.0, 25.0),
                    rng.uniform(-25.0, 25.0),
                    rng.uniform(95.0, 145.0),
                    rng.uniform(-20.0, 20.0),
                    rng.uniform(-180.0, 180.0),
                    rng.uniform(-180.0, 180.0),
                ]
            )
        )

    for x0 in initial_guesses:
        result = least_squares(
            residual,
            x0,
            bounds=(lower, upper),
            max_nfev=5000,
            ftol=1e-11,
            xtol=1e-11,
            gtol=1e-11,
        )

        phase_errors = []
        for m in measurements:
            phase_errors.extend(phase_error_deg(m, result.x, speed_m_s))

        phase_rms = float(np.sqrt(np.mean(np.square(phase_errors))))

        candidate = {
            "cost": float(result.cost),
            "phase_rms_deg": phase_rms,
            "params": result.x.copy(),
            "phase_errors": phase_errors,
        }

        if best is None or candidate["cost"] < best["cost"]:
            best = candidate

    return best


def print_measurements(measurements: list[Measurement]):
    print()
    print("Known-TX measurement summary")
    print(
        "TX   position(mm)      N     A1     A2     A3    "
        "d21 mean±jitter      d31 mean±jitter"
    )
    print("-" * 92)

    for m in measurements:
        assert m.tx_id is not None
        pos = tx_position(m.tx_id)
        print(
            f"{m.tx_id:3d}  ({pos[0]:5.0f},{pos[1]:5.0f})  "
            f"{m.n:5d}  "
            f"{m.amp_mean[0]:5.0f}  {m.amp_mean[1]:5.0f}  {m.amp_mean[2]:5.0f}   "
            f"{m.d21_mean_deg:8.2f} ± {m.d21_jitter_deg:5.2f}   "
            f"{m.d31_mean_deg:8.2f} ± {m.d31_jitter_deg:5.2f}"
        )


def save_calibration(path: str, best, measurements, speed_m_s: float):
    params = best["params"]
    rx = transformed_receivers(params)

    obj = {
        "model": "PAT_3RX_rigid_pose_phase_difference_v1",
        "speed_m_s": speed_m_s,
        "frequency_hz": FREQ_HZ,
        "pat_pitch_mm": PAT_PITCH_MM,
        "nominal_receivers_mm": NOMINAL_RX.tolist(),
        "fit": {
            "dx_mm": float(params[0]),
            "dy_mm": float(params[1]),
            "z_mm": float(params[2]),
            "yaw_deg": float(params[3]),
            "O21_deg": float(params[4]),
            "O31_deg": float(params[5]),
            "phase_rms_deg": float(best["phase_rms_deg"]),
            "fitted_receivers_mm": rx.tolist(),
        },
        "known_tx": [
            {
                "tx_id": m.tx_id,
                "file": m.path,
                "n": m.n,
                "amp_mean_mV_RMS": m.amp_mean.tolist(),
                "d21_mean_deg": m.d21_mean_deg,
                "d31_mean_deg": m.d31_mean_deg,
                "d21_jitter_deg": m.d21_jitter_deg,
                "d31_jitter_deg": m.d31_jitter_deg,
            }
            for m in measurements
        ],
    }

    Path(path).write_text(json.dumps(obj, indent=2))


def load_calibration(path: str):
    obj = json.loads(Path(path).read_text())
    fit = obj["fit"]

    params = np.array(
        [
            fit["dx_mm"],
            fit["dy_mm"],
            fit["z_mm"],
            fit["yaw_deg"],
            fit["O21_deg"],
            fit["O31_deg"],
        ],
        dtype=float,
    )
    return obj, params


def locate_measurement(
    m: Measurement,
    params: np.ndarray,
    speed_m_s: float,
    top_n: int,
):
    candidates = []

    for tx_id in range(256):
        pred21, pred31 = predicted_pair_phases(tx_id, params, speed_m_s)
        e21 = float(wrap_deg(m.d21_mean_deg - pred21))
        e31 = float(wrap_deg(m.d31_mean_deg - pred31))

        # Simple phase-distance score in degrees.
        MIN_RELIABLE_AMP_MV = 100.0

        a1, a2, a3 = m.amp_mean

        pair21_amp = min(a1, a2)
        pair31_amp = min(a1, a3)

        valid21 = pair21_amp >= MIN_RELIABLE_AMP_MV
        valid31 = pair31_amp >= MIN_RELIABLE_AMP_MV

        score_terms = []

        if valid21:
            sigma21 = max(m.d21_jitter_deg, 5.0)
            score_terms.append((e21 / sigma21) ** 2)

        if valid31:
            sigma31 = max(m.d31_jitter_deg, 5.0)
            score_terms.append((e31 / sigma31) ** 2)

        if len(score_terms) == 0:
            phase_error = float("inf")
        else:
            phase_error = math.sqrt(sum(score_terms))

        candidates.append(
            {
                "tx_id": tx_id,
                "position": tx_position(tx_id),
                "phase_error_deg": phase_error,
                "e21_deg": e21,
                "e31_deg": e31,
                "pred21_deg": pred21,
                "pred31_deg": pred31,
            }
        )

    candidates.sort(key=lambda x: x["phase_error_deg"])
    return candidates[:top_n]


def command_calibrate(args):
    measurements = [
        read_measurement(path, tx_id)
        for tx_id, path in args.sample
    ]

    print_measurements(measurements)

    if len(measurements) < 8:
        print()
        print(
            "WARNING: only "
            f"{len(measurements)} calibration TX positions were supplied. "
            "The 6-parameter wrapped-phase fit is only provisional. "
            "Use 8-12 well-distributed TX positions before trusting blind location."
        )

    best = fit_calibration(
        measurements,
        speed_m_s=args.speed,
        starts=args.starts,
        seed=args.seed,
    )

    params = best["params"]
    rx = transformed_receivers(params)

    print()
    print("Provisional calibration fit")
    print(f"phase RMS residual : {best['phase_rms_deg']:.3f} deg")
    print(f"dx                 : {params[0]:.3f} mm")
    print(f"dy                 : {params[1]:.3f} mm")
    print(f"z                  : {params[2]:.3f} mm")
    print(f"yaw                : {params[3]:.3f} deg")
    print(f"O21                : {params[4]:.3f} deg")
    print(f"O31                : {params[5]:.3f} deg")
    print()
    print("Fitted receiver centers:")
    for i, xyz in enumerate(rx, 1):
        print(f"RX{i}: ({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}) mm")

    print()
    print("Per-known-TX residual:")
    for m in measurements:
        e21, e31 = phase_error_deg(m, params, args.speed)
        print(
            f"TX{m.tx_id:3d}: "
            f"e21={e21:+7.2f} deg, "
            f"e31={e31:+7.2f} deg, "
            f"|e|={math.hypot(e21,e31):6.2f} deg"
        )

    save_calibration(args.output, best, measurements, args.speed)
    print()
    print(f"Saved calibration: {args.output}")


def command_locate(args):
    obj, params = load_calibration(args.calibration)
    m = read_measurement(args.csv, None)

    print()
    print("Unknown measurement")
    print(f"file       : {args.csv}")
    print(f"samples    : {m.n}")
    print(
        f"amplitude  : "
        f"CH1={m.amp_mean[0]:.1f}, "
        f"CH2={m.amp_mean[1]:.1f}, "
        f"CH3={m.amp_mean[2]:.1f} mV RMS"
    )
    print(
        f"d21        : {m.d21_mean_deg:.3f} deg "
        f"(jitter {m.d21_jitter_deg:.3f})"
    )
    print(
        f"d31        : {m.d31_mean_deg:.3f} deg "
        f"(jitter {m.d31_jitter_deg:.3f})"
    )

    candidates = locate_measurement(
        m,
        params,
        speed_m_s=float(obj["speed_m_s"]),
        top_n=args.top,
    )

    print()
    print("Best PAT-grid candidates")
    print("rank  TX     x      y     phase_err     e21      e31")
    print("-" * 63)
    for rank, c in enumerate(candidates, 1):
        pos = c["position"]
        print(
            f"{rank:4d}  {c['tx_id']:3d}  "
            f"{pos[0]:5.0f}  {pos[1]:5.0f}   "
            f"{c['phase_error_deg']:8.2f}   "
            f"{c['e21_deg']:+7.2f}  "
            f"{c['e31_deg']:+7.2f}"
        )


def build_parser():
    parser = argparse.ArgumentParser(
        description="PAT 3-RX phase calibration and transducer triangulation."
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p_cal = sub.add_parser(
        "calibrate",
        help="Fit receiver pose + pair phase offsets from known TX CSVs.",
    )
    p_cal.add_argument(
        "--sample",
        action="append",
        type=parse_sample_arg,
        required=True,
        metavar="TXID=CSV",
        help="Known TX sample. Repeat this option for every calibration file.",
    )
    p_cal.add_argument(
        "--speed",
        type=float,
        default=343.0,
        help="Speed of sound in m/s (default 343.0).",
    )
    p_cal.add_argument(
        "--starts",
        type=int,
        default=150,
        help="Number of multi-start fits (default 150).",
    )
    p_cal.add_argument("--seed", type=int, default=1)
    p_cal.add_argument(
        "--output",
        default="sample_log/pat_phase_calibration.json",
    )
    p_cal.set_defaults(func=command_calibrate)

    p_loc = sub.add_parser(
        "locate",
        help="Rank the 256 PAT TX positions for an unknown CSV.",
    )
    p_loc.add_argument(
        "--calibration",
        default="sample_log/pat_phase_calibration.json",
    )
    p_loc.add_argument("--csv", required=True)
    p_loc.add_argument("--top", type=int, default=10)
    p_loc.set_defaults(func=command_locate)

    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
