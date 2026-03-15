"""
preprocess.py — Load internal.csv files, label modes, generate one-step training tables.

Usage:
    python preprocess.py [--results-dir PATH] [--t-settle 0.5] [--out-dir PATH]

Output:
    out_dir/train_table.parquet   — one-step supervised table (run_id, mode, s_t, u_t, s_{t+1})
    out_dir/summary.txt          — per-run/per-mode sample counts
"""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd

# Column layout of 15-col internal.csv
CSV_COLUMNS = [
    "time", "x_des", "y_des", "x_meas", "y_meas", "z_meas",
    "ex", "ey", "vx", "vy", "vz",
    "fz_des", "fz_meas", "fz_cmd", "phase",
]

# Koopman state (9-dim) and input (3-dim)
STATE_COLS = ["x_meas", "y_meas", "z_meas", "vx", "vy", "vz", "ex", "ey", "fz_meas"]
INPUT_COLS = ["x_des", "y_des", "fz_des"]


def find_csv_files(results_dir: Path, min_cols: int = 15) -> list[Path]:
    """Find all internal.csv with at least min_cols columns."""
    csvs = sorted(results_dir.rglob("internal.csv"))
    valid = []
    for p in csvs:
        with open(p) as f:
            header = f.readline().strip()
        if len(header.split(",")) >= min_cols:
            valid.append(p)
    return valid


def load_run(csv_path: Path) -> pd.DataFrame:
    """Load a single internal.csv."""
    df = pd.read_csv(csv_path, names=CSV_COLUMNS, header=0)
    return df


def label_modes(df: pd.DataFrame, t_settle: float) -> pd.Series:
    """
    Label each row as M0 / M1 / M2.

    M0: phase == 0  (pre-contact)
    M1: phase == 1 AND t_since_contact < t_settle  (transition)
    M2: phase == 1 AND t_since_contact >= t_settle  (steady contact)
    """
    modes = pd.Series("M0", index=df.index)

    contact_mask = df["phase"] == 1
    if not contact_mask.any():
        return modes

    first_contact_idx = contact_mask.idxmax()
    t_contact = df.loc[first_contact_idx, "time"]

    contact_rows = df.index[contact_mask]
    t_since = df.loc[contact_rows, "time"] - t_contact

    modes.loc[contact_rows[t_since < t_settle]] = "M1"
    modes.loc[contact_rows[t_since >= t_settle]] = "M2"

    return modes


def build_training_table(
    csv_files: list[Path], t_settle: float
) -> pd.DataFrame:
    """
    Build one-step supervised table from all runs.

    Each row: run_id, mode, s_t (9 cols), u_t (3 cols), s_{t+1} (9 cols)
    """
    all_rows = []

    for run_id, csv_path in enumerate(csv_files):
        df = load_run(csv_path)
        modes = label_modes(df, t_settle)

        s = df[STATE_COLS].values  # (T, 9)
        u = df[INPUT_COLS].values  # (T, 3)
        m = modes.values           # (T,)

        # One-step pairs: (s_t, u_t, mode_t) -> s_{t+1}
        # mode is assigned from t (not t+1)
        n = len(df) - 1
        s_t = s[:n]
        u_t = u[:n]
        s_next = s[1:]
        mode_t = m[:n]

        run_df = pd.DataFrame(
            np.column_stack([
                np.full(n, run_id),
                mode_t.reshape(-1, 1),
                s_t,
                u_t,
                s_next,
            ]),
            columns=(
                ["run_id", "mode"]
                + [f"s_{c}" for c in STATE_COLS]
                + [f"u_{c}" for c in INPUT_COLS]
                + [f"sn_{c}" for c in STATE_COLS]
            ),
        )
        all_rows.append(run_df)

    table = pd.concat(all_rows, ignore_index=True)
    table["run_id"] = table["run_id"].astype(int)
    return table


def print_summary(table: pd.DataFrame, csv_files: list[Path]) -> str:
    lines = []
    lines.append(f"Total runs: {len(csv_files)}")
    lines.append(f"Total samples: {len(table)}")
    lines.append("")
    lines.append("Per-mode counts:")
    for mode in ["M0", "M1", "M2"]:
        count = (table["mode"] == mode).sum()
        lines.append(f"  {mode}: {count}")
    lines.append("")
    lines.append("Per-run counts:")
    for run_id in sorted(table["run_id"].unique()):
        run_mask = table["run_id"] == run_id
        n = run_mask.sum()
        csv_name = csv_files[int(run_id)].parent.name
        lines.append(f"  run {run_id} ({csv_name}): {n}")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description="Koopman preprocess")
    parser.add_argument(
        "--results-dir",
        type=Path,
        default=Path("/home/andy/franka_ros2_ws/src/results/hybrid_circle_force"),
    )
    parser.add_argument("--t-settle", type=float, default=0.5)
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("/home/andy/franka_ros2_ws/src/koopman/data"),
    )
    args = parser.parse_args()

    csv_files = find_csv_files(args.results_dir)
    if not csv_files:
        print(f"ERROR: No 15-col internal.csv found in {args.results_dir}")
        return

    print(f"Found {len(csv_files)} valid runs")

    table = build_training_table(csv_files, args.t_settle)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    out_path = args.out_dir / f"train_table_settle{args.t_settle}.csv"
    table.to_csv(out_path, index=False)
    print(f"Saved: {out_path}")

    summary = print_summary(table, csv_files)
    print(summary)

    summary_path = args.out_dir / f"summary_settle{args.t_settle}.txt"
    summary_path.write_text(summary)


if __name__ == "__main__":
    main()
