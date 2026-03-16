#!/usr/bin/env python3
# Post-match shooter analysis script for FRC Team 5928
#
# Dependencies: matplotlib, numpy, scipy
# Install with: pip install matplotlib numpy scipy
#
# Usage:
#   python scripts/analyze.py fetch [--rio <ip>] [--dest <dir>]
#   python scripts/analyze.py plot <output_data_file> [--table <lookup_table>]
#   python scripts/analyze.py fit <output_data_file> [--table <lookup_table>] [--out <output_table>]

import argparse
import csv
import os
import subprocess
import sys

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

DEFAULT_RIO_IP = "10.59.28.2"
RIO_SHOTS_DIR = "/home/lvuser/match_shots/"
RIO_USER = "lvuser"
DEFAULT_LOCAL_DEST = "match_data"
DEFAULT_LOOKUP_TABLE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "main", "deploy", "shooter_lookup_table.csv",
)

# CSV columns in the Output_Data_File
ODF_COLS = [
    "matchTimestamp",
    "distanceMeters",
    "rotationToTargetDeg",
    "computedShotAngleDeg",
    "actualShotAngleDeg",
    "targetHoodAngleDeg",
    "currentHoodAngleDeg",
    "targetFlywheelRPS",
    "currentFlywheelRPS",
]

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def load_output_data(path):
    """Return list of dicts from an Output_Data_File CSV."""
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({k: float(v) for k, v in row.items()})
    return rows


def load_lookup_table(path):
    """Return (distances, hood_angles, flywheel_rps) arrays from the lookup table CSV."""
    import numpy as np
    distances, hoods, flywheels = [], [], []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            distances.append(float(row["distance_m"]))
            hoods.append(float(row["hood_angle_deg"]))
            flywheels.append(float(row["flywheel_rps"]))
    return np.array(distances), np.array(hoods), np.array(flywheels)


def write_lookup_table(path, distances, hoods, flywheels):
    """Write arrays back to the lookup table CSV format."""
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["distance_m", "hood_angle_deg", "flywheel_rps"])
        for d, h, r in zip(distances, hoods, flywheels):
            writer.writerow([round(d, 6), round(h, 4), round(r, 4)])


# ---------------------------------------------------------------------------
# Sub-command: fetch
# ---------------------------------------------------------------------------

def cmd_fetch(args):
    """Copy all files from the RIO's match_shots directory to a local folder."""
    rio_src = f"{RIO_USER}@{args.rio}:{RIO_SHOTS_DIR}"
    dest = args.dest
    os.makedirs(dest, exist_ok=True)

    # Use scp -r to copy the whole directory contents
    scp_cmd = ["scp", "-r", rio_src, dest]
    print(f"Running: {' '.join(scp_cmd)}")

    result = subprocess.run(scp_cmd, capture_output=True, text=True)

    if result.returncode != 0:
        stderr = result.stderr.strip()
        # scp exits non-zero when the source directory is empty or doesn't exist
        if "No such file or directory" in stderr or "not a regular file" in stderr:
            print("No match data files found on the RIO.")
            sys.exit(0)
        print(f"scp error: {stderr}", file=sys.stderr)
        sys.exit(result.returncode)

    # List what was copied
    copied = []
    for root, _dirs, files in os.walk(dest):
        for fname in files:
            fpath = os.path.join(root, fname)
            size = os.path.getsize(fpath)
            copied.append((fname, size))

    if not copied:
        print("No match data files found on the RIO.")
        sys.exit(0)

    print(f"Fetched {len(copied)} file(s) to '{dest}':")
    for fname, size in sorted(copied):
        print(f"  {fname}  ({size} bytes)")


# ---------------------------------------------------------------------------
# Sub-command: plot
# ---------------------------------------------------------------------------

def cmd_plot(args):
    """Scatter-plot distance vs. actual shot angle and flywheel speed."""
    import numpy as np
    import matplotlib.pyplot as plt

    rows = load_output_data(args.file)
    if not rows:
        print("No data rows found in the file.")
        sys.exit(0)

    distances = np.array([r["distanceMeters"] for r in rows])
    actual_angles = np.array([r["actualShotAngleDeg"] for r in rows])
    actual_rps = np.array([r["currentFlywheelRPS"] for r in rows])

    # Load lookup table for overlay
    tbl_distances, tbl_hoods, tbl_flywheels = load_lookup_table(args.table)
    # Sort by distance for a clean line
    sort_idx = np.argsort(tbl_distances)
    tbl_distances = tbl_distances[sort_idx]
    tbl_hoods = tbl_hoods[sort_idx]
    tbl_flywheels = tbl_flywheels[sort_idx]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    fig.suptitle(f"Match Shot Analysis\n{os.path.basename(args.file)}")

    # --- Hood angle plot ---
    ax1.scatter(distances, actual_angles, color="steelblue", label="Actual shot angle", zorder=3)
    ax1.plot(tbl_distances, tbl_hoods, color="orange", linewidth=2, label="Lookup table", zorder=2)
    ax1.set_ylabel("Hood Angle (deg)")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # --- Flywheel RPS plot ---
    ax2.scatter(distances, actual_rps, color="steelblue", label="Actual flywheel RPS", zorder=3)
    ax2.plot(tbl_distances, tbl_flywheels, color="orange", linewidth=2, label="Lookup table", zorder=2)
    ax2.set_xlabel("Distance (m)")
    ax2.set_ylabel("Flywheel Speed (RPS)")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# Sub-command: fit
# ---------------------------------------------------------------------------

def cmd_fit(args):
    """Fit piecewise-linear curves through recorded data and write updated lookup table."""
    import numpy as np

    rows = load_output_data(args.file)
    if not rows:
        print("No data rows found in the file.")
        sys.exit(0)

    distances = np.array([r["distanceMeters"] for r in rows])
    actual_angles = np.array([r["actualShotAngleDeg"] for r in rows])
    actual_rps = np.array([r["currentFlywheelRPS"] for r in rows])

    # Sort by distance for interpolation
    sort_idx = np.argsort(distances)
    distances = distances[sort_idx]
    actual_angles = actual_angles[sort_idx]
    actual_rps = actual_rps[sort_idx]

    # Load existing lookup table to get the canonical distance knots
    tbl_distances, _tbl_hoods, _tbl_flywheels = load_lookup_table(args.table)
    tbl_distances_sorted = np.sort(tbl_distances)

    # Fit: use the recorded data distances as the new knot points.
    # Deduplicate distances by averaging values at the same distance.
    unique_dists, inv = np.unique(distances, return_inverse=True)
    avg_angles = np.array([actual_angles[inv == i].mean() for i in range(len(unique_dists))])
    avg_rps = np.array([actual_rps[inv == i].mean() for i in range(len(unique_dists))])

    # New lookup table knots: union of existing table distances and recorded distances,
    # clipped to the range of recorded data so we don't extrapolate wildly.
    d_min, d_max = unique_dists[0], unique_dists[-1]
    extra_knots = tbl_distances_sorted[
        (tbl_distances_sorted >= d_min) & (tbl_distances_sorted <= d_max)
    ]
    new_knots = np.unique(np.concatenate([unique_dists, extra_knots]))

    # Interpolate the fitted curves at the new knots
    new_hoods = np.interp(new_knots, unique_dists, avg_angles)
    new_rps = np.interp(new_knots, unique_dists, avg_rps)

    # Write updated lookup table
    out_path = args.out
    write_lookup_table(out_path, new_knots, new_hoods, new_rps)
    print(f"Updated lookup table written to: {out_path}")
    print(f"  {len(new_knots)} knot points")

    # Re-project original data points onto the new curve
    print("\nOriginal data re-projected onto new curve:")
    print(f"{'Distance (m)':>14}  {'Actual Angle':>14}  {'Fitted Angle':>14}  {'Actual RPS':>12}  {'Fitted RPS':>12}")
    print("-" * 72)
    for d, a_ang, a_rps in zip(distances, actual_angles, actual_rps):
        f_ang = float(np.interp(d, new_knots, new_hoods))
        f_rps = float(np.interp(d, new_knots, new_rps))
        print(f"{d:>14.4f}  {a_ang:>14.4f}  {f_ang:>14.4f}  {a_rps:>12.4f}  {f_rps:>12.4f}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Post-match shooter analysis for FRC Team 5928"
    )
    sub = parser.add_subparsers(dest="command", required=True)

    # --- fetch ---
    p_fetch = sub.add_parser("fetch", help="Copy match data files from the RIO")
    p_fetch.add_argument(
        "--rio", default=DEFAULT_RIO_IP,
        help=f"RIO IP address (default: {DEFAULT_RIO_IP})"
    )
    p_fetch.add_argument(
        "--dest", default=DEFAULT_LOCAL_DEST,
        help=f"Local destination directory (default: {DEFAULT_LOCAL_DEST})"
    )

    # --- plot ---
    p_plot = sub.add_parser("plot", help="Plot distance vs. shot angle and flywheel speed")
    p_plot.add_argument("file", help="Path to the Output_Data_File CSV")
    p_plot.add_argument(
        "--table", default=DEFAULT_LOOKUP_TABLE,
        help="Path to the lookup table CSV (default: src/main/deploy/shooter_lookup_table.csv)"
    )

    # --- fit ---
    p_fit = sub.add_parser("fit", help="Fit new lookup table curve from recorded data")
    p_fit.add_argument("file", help="Path to the Output_Data_File CSV")
    p_fit.add_argument(
        "--table", default=DEFAULT_LOOKUP_TABLE,
        help="Path to the lookup table CSV (default: src/main/deploy/shooter_lookup_table.csv)"
    )
    p_fit.add_argument(
        "--out", default=DEFAULT_LOOKUP_TABLE,
        help="Output path for updated lookup table (default: overwrites src/main/deploy/shooter_lookup_table.csv)"
    )

    args = parser.parse_args()

    if args.command == "fetch":
        cmd_fetch(args)
    elif args.command == "plot":
        cmd_plot(args)
    elif args.command == "fit":
        cmd_fit(args)


if __name__ == "__main__":
    main()
