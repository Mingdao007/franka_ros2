"""
plot_spectrum.py — Eigenvalue visualization for EDMD Koopman operators.

Usage:
    python plot_spectrum.py [--data-dir PATH] [--t-settle 0.5]

Plots eigenvalue spectra (unit circle + real/imaginary) for each mode's K matrix.
"""

import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


def load_edmd_models(data_dir: Path, t_settle: float) -> dict:
    path = data_dir / f"edmd_models_settle{t_settle}.npz"
    data = np.load(path, allow_pickle=True)
    models = {}
    for key in data.files:
        if key.startswith("edmd_K_"):
            name = key.replace("edmd_K_", "")
            models[name] = data[key]
    return models


def plot_eigenvalues(models: dict, out_dir: Path, t_settle: float):
    """Plot eigenvalue spectra on the unit circle for each mode."""
    fig, axes = plt.subplots(1, len(models), figsize=(5 * len(models), 5))
    if len(models) == 1:
        axes = [axes]

    colors = {"M0": "blue", "M1": "red", "M2": "green"}

    for ax, (name, K) in zip(axes, models.items()):
        # K is (9, 20) — extract the (9,9) linear-state block for eigenvalue analysis
        K_linear = K[:, :9]
        eigs = np.linalg.eigvals(K_linear)

        color = colors.get(name, "black")

        # Unit circle
        theta = np.linspace(0, 2 * np.pi, 200)
        ax.plot(np.cos(theta), np.sin(theta), "k-", alpha=0.3, linewidth=0.5)

        ax.scatter(eigs.real, eigs.imag, c=color, s=60, zorder=5, label=name)
        ax.set_xlabel("Real")
        ax.set_ylabel("Imaginary")
        ax.set_title(f"{name} (T_settle={t_settle}s)")
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
        ax.axhline(0, color="k", linewidth=0.5)
        ax.axvline(0, color="k", linewidth=0.5)

        # Annotate eigenvalue magnitudes
        mags = np.abs(eigs)
        ax.set_xlim(-1.5, 1.5)
        ax.set_ylim(-1.5, 1.5)

        # Print eigenvalue summary
        print(f"\n{name} eigenvalues:")
        for i, e in enumerate(sorted(eigs, key=lambda x: -abs(x))):
            print(f"  λ_{i}: {e.real:+.4f} {e.imag:+.4f}j  |λ|={abs(e):.4f}")

    plt.tight_layout()
    out_path = out_dir / f"eigenvalues_settle{t_settle}.png"
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f"\nPlot saved: {out_path}")

    # Combined overlay plot
    fig, ax = plt.subplots(figsize=(6, 6))
    theta = np.linspace(0, 2 * np.pi, 200)
    ax.plot(np.cos(theta), np.sin(theta), "k-", alpha=0.3, linewidth=0.5)

    for name, K in models.items():
        K_linear = K[:, :9]
        eigs = np.linalg.eigvals(K_linear)
        color = colors.get(name, "black")
        ax.scatter(eigs.real, eigs.imag, c=color, s=60, alpha=0.7, label=name, zorder=5)

    ax.set_xlabel("Real")
    ax.set_ylabel("Imaginary")
    ax.set_title(f"All modes — eigenvalue overlay (T_settle={t_settle}s)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)

    out_path = out_dir / f"eigenvalues_overlay_settle{t_settle}.png"
    plt.savefig(out_path, dpi=150)
    plt.close()
    print(f"Overlay plot saved: {out_path}")


def main():
    parser = argparse.ArgumentParser(description="Koopman eigenvalue plots")
    parser.add_argument(
        "--data-dir", type=Path,
        default=Path("/home/andy/franka_ros2_ws/src/koopman/data"),
    )
    parser.add_argument("--t-settle", type=float, default=0.5)
    args = parser.parse_args()

    models = load_edmd_models(args.data_dir, args.t_settle)
    if not models:
        print("No EDMD models found. Run edmd.py first.")
        return

    plot_eigenvalues(models, args.data_dir, args.t_settle)


if __name__ == "__main__":
    main()
