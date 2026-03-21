#!/usr/bin/env python3
"""Per-event contact transient analysis for C1 ablation experiments.

Usage:
  python3 analyze_contact_events.py --results-dir src/results/exp_c1_ablation

Metrics (per re-contact event, 1.0s window after phase 0->1 transition):
  - IAE: integral of |Fz_des - Fz_meas| over window
  - Peak: max |Fz_des - Fz_meas| in window
  - Settle: time to enter |error| < threshold AND hold for hold_duration

Reports event 1 (cold start) separately, then median of events 2..N.
"""

import argparse
import csv
import math
import os
import statistics


SETTLE_THRESHOLD = 0.5   # N
SETTLE_HOLD = 0.050      # s (50ms continuous hold)
EVENT_WINDOW = 1.0       # s


def load_csv(path):
    data = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                t = float(row['time'])
                phase = int(row.get('phase', ''))
                fz_des = float(row['fz_des'])
                fz_meas = float(row['fz_meas'])
            except (ValueError, TypeError, KeyError):
                continue
            data.append((t, phase, fz_des, fz_meas))
    return data


def find_contact_events(data):
    """Find indices where phase transitions from 0 (descent) to 1 (circle)."""
    return [i for i in range(1, len(data)) if data[i-1][1] == 0 and data[i][1] == 1]


def analyze_event(data, start_idx):
    t0 = data[start_idx][0]

    # IAE and peak within event window
    iae = 0.0
    peak = 0.0
    t_prev = None
    for j in range(start_idx, len(data)):
        t, _, fz_des, fz_meas = data[j]
        if t - t0 > EVENT_WINDOW:
            break
        dt = (t - t_prev) if t_prev is not None else 0.001
        t_prev = t
        err = abs(fz_des - fz_meas)
        iae += err * dt
        peak = max(peak, err)

    # Settle: first time |error| stays below threshold for SETTLE_HOLD
    settle = None
    for j in range(start_idx, len(data)):
        t, _, fz_des, fz_meas = data[j]
        if t - t0 > 2.0:
            break
        if abs(fz_des - fz_meas) < SETTLE_THRESHOLD:
            t_enter = t
            held = True
            for k in range(j, len(data)):
                if data[k][0] - t_enter > SETTLE_HOLD:
                    break
                if abs(data[k][2] - data[k][3]) >= SETTLE_THRESHOLD:
                    held = False
                    break
            if held:
                settle = t_enter - t0
                break

    return {'iae': iae, 'peak': peak, 'settle': settle}


def analyze_file(path, label):
    data = load_csv(path)
    if not data:
        print(f"\n  {label}: NO DATA")
        return None

    indices = find_contact_events(data)
    if not indices:
        print(f"\n  {label}: NO CONTACT EVENTS")
        return None

    results = [analyze_event(data, idx) for idx in indices]

    print(f"\n  {label}  ({len(results)} events)")
    print(f"  {'#':>4}  {'IAE':>8}  {'peak':>7}  {'settle':>10}")
    print(f"  " + "-" * 35)
    for i, r in enumerate(results):
        s = f"{r['settle']*1000:.0f}ms" if r['settle'] is not None else ">2000ms"
        print(f"  {i+1:>4}  {r['iae']:>8.4f}  {r['peak']:>7.3f}  {s:>10}")

    if len(results) > 1:
        rest = results[1:]
        iaes = [r['iae'] for r in rest]
        peaks = [r['peak'] for r in rest]
        settles = [r['settle'] for r in rest if r['settle'] is not None]
        no_settle = sum(1 for r in rest if r['settle'] is None)

        print(f"\n  Events 2..{len(results)} (excluding cold start):")
        print(f"    IAE   median={statistics.median(iaes):.4f}  mean={statistics.mean(iaes):.4f}")
        print(f"    Peak  median={statistics.median(peaks):.3f}N  mean={statistics.mean(peaks):.3f}N")
        if settles:
            print(f"    Settle median={statistics.median(settles)*1000:.0f}ms  mean={statistics.mean(settles)*1000:.0f}ms")
        if no_settle:
            print(f"    No-settle (>2s): {no_settle}/{len(rest)} events")

    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--results-dir", required=True)
    args = parser.parse_args()

    print("=" * 65)
    print(f"  Per-Event Contact Analysis")
    print(f"  Window={EVENT_WINDOW}s  Settle: |err|<{SETTLE_THRESHOLD}N held {SETTLE_HOLD*1000:.0f}ms")
    print("=" * 65)

    files = sorted(f for f in os.listdir(args.results_dir) if f.endswith('.csv'))
    all_results = {}
    for f in files:
        label = f.replace('.csv', '')
        path = os.path.join(args.results_dir, f)
        r = analyze_file(path, label)
        if r and len(r) > 1:
            rest = r[1:]
            all_results[label] = {
                'iae': statistics.median([x['iae'] for x in rest]),
                'peak': statistics.median([x['peak'] for x in rest]),
                'settle': statistics.median([x['settle'] for x in rest if x['settle'] is not None]) * 1000
                          if any(x['settle'] is not None for x in rest) else float('nan'),
                'no_settle': sum(1 for x in rest if x['settle'] is None),
            }

    if all_results:
        print("\n" + "=" * 65)
        print("  Summary (events 2..N, median)")
        print("=" * 65)
        for label, r in all_results.items():
            ns = f" +{r['no_settle']}ns" if r['no_settle'] else ""
            print(f"  {label:30s}  IAE={r['iae']:.4f}  peak={r['peak']:.3f}N  settle={r['settle']:.0f}ms{ns}")


if __name__ == "__main__":
    main()
