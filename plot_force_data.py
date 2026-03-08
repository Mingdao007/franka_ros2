#!/usr/bin/env python3
"""
Force Data Plotter
==================
Reads CSV data and generates static force tracking plot.

Usage:
    python3 plot_force_data.py <csv_file> [--output plot.png]
    python3 plot_force_data.py --latest  # Plot most recent CSV file
"""
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import argparse
import glob
import os
from datetime import datetime


def find_latest_csv(directory='/home/andy/franka_ros2_ws'):
    """Find the most recently modified CSV file"""
    pattern = os.path.join(directory, 'force_data_*.csv')
    files = glob.glob(pattern)
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def plot_force_tracking(csv_file, output_file=None, show=True):
    """Generate force tracking plot from CSV data"""
    
    # Read data
    df = pd.read_csv(csv_file)
    
    if df.empty:
        print(f'Error: Empty data file: {csv_file}')
        return None
    
    # Setup plot style
    plt.style.use('default')
    plt.rcParams.update({
        'font.size': 14,
        'axes.titlesize': 16,
        'axes.labelsize': 14,
        'legend.fontsize': 12,
        'xtick.labelsize': 12,
        'ytick.labelsize': 12,
    })
    
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    ax_fx, ax_fy, ax_fz = axes
    
    # Title with timestamp from filename
    basename = os.path.basename(csv_file)
    fig.suptitle(f'Force Tracking Result\n{basename}', fontsize=18, fontweight='bold')
    
    t = df['time']
    
    # Plot Fx
    ax_fx.plot(t, df['cmd_fx'], 'b-', linewidth=2.5, label='Commanded')
    ax_fx.plot(t, df['est_fx'], 'r-', linewidth=2, alpha=0.8, label='Estimated')
    ax_fx.set_ylabel('Force X (N)')
    ax_fx.legend(loc='upper right')
    ax_fx.grid(True, alpha=0.3)
    
    # Plot Fy
    ax_fy.plot(t, df['cmd_fy'], 'b-', linewidth=2.5, label='Commanded')
    ax_fy.plot(t, df['est_fy'], 'r-', linewidth=2, alpha=0.8, label='Estimated')
    ax_fy.set_ylabel('Force Y (N)')
    ax_fy.legend(loc='upper right')
    ax_fy.grid(True, alpha=0.3)
    
    # Plot Fz
    ax_fz.plot(t, df['cmd_fz'], 'b-', linewidth=2.5, label='Commanded')
    ax_fz.plot(t, df['est_fz'], 'r-', linewidth=2, alpha=0.8, label='Estimated')
    ax_fz.set_xlabel('Time (s)')
    ax_fz.set_ylabel('Force Z (N)')
    ax_fz.legend(loc='upper right')
    ax_fz.grid(True, alpha=0.3)
    
    # Set x-axis from 0 to max time
    ax_fz.set_xlim(0, t.max())
    
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    
    # Save if output specified
    if output_file:
        fig.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f'[SAVED] {output_file}')
    else:
        # Auto-generate output filename
        out_path = csv_file.replace('.csv', '.png')
        fig.savefig(out_path, dpi=150, bbox_inches='tight')
        print(f'[SAVED] {out_path}')
    
    if show:
        plt.show()
    
    return fig


def main():
    parser = argparse.ArgumentParser(description='Force Data Plotter')
    parser.add_argument('csv_file', nargs='?', default=None,
                       help='CSV file to plot')
    parser.add_argument('--latest', action='store_true',
                       help='Plot the most recent CSV file')
    parser.add_argument('--output', type=str, default=None,
                       help='Output PNG file path')
    parser.add_argument('--no-show', action='store_true',
                       help='Do not display the plot')
    args = parser.parse_args()
    
    # Determine input file
    if args.latest:
        csv_file = find_latest_csv()
        if csv_file is None:
            print('Error: No force_data_*.csv files found')
            return
        print(f'Using latest file: {csv_file}')
    elif args.csv_file:
        csv_file = args.csv_file
    else:
        # Try to find latest
        csv_file = find_latest_csv()
        if csv_file is None:
            print('Usage: python3 plot_force_data.py <csv_file>')
            print('   or: python3 plot_force_data.py --latest')
            return
        print(f'Using latest file: {csv_file}')
    
    if not os.path.exists(csv_file):
        print(f'Error: File not found: {csv_file}')
        return
    
    plot_force_tracking(csv_file, args.output, show=not args.no_show)


if __name__ == '__main__':
    main()
