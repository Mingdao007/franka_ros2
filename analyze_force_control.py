#!/usr/bin/env python3
"""
Force Control Performance Analysis Script
==========================================
Analyzes force control data from internal_force_log.csv
Calculates:
1. Steady-state error
2. Amplitude ratio (for sine wave tracking)
3. Phase lag
4. Pass/Fail criteria based on PI standards
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal
from scipy.optimize import curve_fit
import os

# PI Standards for validation
PASS_CRITERIA = {
    'steady_state_error': {'pass': 1.0, 'excellent': 0.2, 'unit': 'N'},
    'phase_lag_0p5hz': {'pass': 30.0, 'excellent': 5.0, 'unit': '°'},  # degrees
    'amplitude_ratio_5hz': {'pass': 0.5, 'excellent': 0.9, 'unit': ''},  # ratio
}

def load_data(filepath):
    """Load force control log data"""
    df = pd.read_csv(filepath)
    return df

def sine_fit(t, A, omega, phi, offset):
    """Sine wave model for curve fitting"""
    return A * np.sin(omega * t + phi) + offset

def analyze_steady_state(df, start_time=1.0):
    """
    Analyze steady-state error (for constant force tracking)
    Assumes force_amplitude = 0 or very small
    """
    # Filter data after transient
    mask = df['time'] > start_time
    f_des = df['f_des'][mask]
    f_meas = df['f_meas'][mask]
    
    error = f_des - f_meas
    mean_error = np.mean(error)
    std_error = np.std(error)
    max_error = np.max(np.abs(error))
    
    return {
        'mean_error': mean_error,
        'std_error': std_error,
        'max_error': max_error,
        'rms_error': np.sqrt(np.mean(error**2))
    }

def analyze_sine_tracking(df, freq, start_time=2.0):
    """
    Analyze sine wave tracking performance
    Returns amplitude ratio and phase lag
    """
    # Filter data after transient
    mask = df['time'] > start_time
    t = df['time'][mask].values
    f_des = df['f_des'][mask].values
    f_meas = df['f_meas'][mask].values
    
    if len(t) < 100:
        return {'error': 'Not enough data points'}
    
    omega = 2 * np.pi * freq
    
    try:
        # Fit sine to commanded force
        p0_cmd = [2.0, omega, 0, 5.0]  # Initial guess
        popt_cmd, _ = curve_fit(sine_fit, t, f_des, p0=p0_cmd, maxfev=10000)
        A_cmd = abs(popt_cmd[0])
        phi_cmd = popt_cmd[2]
        
        # Fit sine to measured force
        p0_meas = [2.0, omega, 0, 5.0]
        popt_meas, _ = curve_fit(sine_fit, t, f_meas, p0=p0_meas, maxfev=10000)
        A_meas = abs(popt_meas[0])
        phi_meas = popt_meas[2]
        
        # Calculate amplitude ratio
        amplitude_ratio = A_meas / A_cmd if A_cmd > 0.01 else 0
        
        # Calculate phase lag (in degrees)
        phase_diff = phi_cmd - phi_meas
        # Normalize to [-180, 180]
        while phase_diff > np.pi:
            phase_diff -= 2 * np.pi
        while phase_diff < -np.pi:
            phase_diff += 2 * np.pi
        phase_lag_deg = np.degrees(phase_diff)
        
        # Alternative: Cross-correlation for phase lag
        correlation = np.correlate(f_des - np.mean(f_des), 
                                   f_meas - np.mean(f_meas), mode='full')
        dt = t[1] - t[0] if len(t) > 1 else 0.001
        lags = np.arange(-len(f_des)+1, len(f_des)) * dt
        peak_idx = np.argmax(correlation)
        time_lag = lags[peak_idx]
        phase_lag_xcorr = time_lag * freq * 360  # degrees
        
        return {
            'amplitude_cmd': A_cmd,
            'amplitude_meas': A_meas,
            'amplitude_ratio': amplitude_ratio,
            'amplitude_attenuation_percent': (1 - amplitude_ratio) * 100,
            'phase_lag_deg': phase_lag_deg,
            'phase_lag_xcorr_deg': phase_lag_xcorr,
            'frequency': freq
        }
    except Exception as e:
        return {'error': str(e)}

def check_pass_fail(results, freq):
    """Check if results meet PI standards"""
    verdicts = {}
    
    if 'steady_state' in results:
        ss = results['steady_state']
        err = abs(ss['mean_error'])
        if err < PASS_CRITERIA['steady_state_error']['excellent']:
            verdicts['steady_state'] = ('EXCELLENT ✓', err)
        elif err < PASS_CRITERIA['steady_state_error']['pass']:
            verdicts['steady_state'] = ('PASS ✓', err)
        else:
            verdicts['steady_state'] = ('FAIL ✗', err)
    
    if 'sine_tracking' in results:
        st = results['sine_tracking']
        if 'error' not in st:
            # Phase lag check (for 0.5 Hz)
            if freq <= 0.5:
                lag = abs(st['phase_lag_xcorr_deg'])
                if lag < PASS_CRITERIA['phase_lag_0p5hz']['excellent']:
                    verdicts['phase_lag'] = ('EXCELLENT ✓', lag)
                elif lag < PASS_CRITERIA['phase_lag_0p5hz']['pass']:
                    verdicts['phase_lag'] = ('PASS ✓', lag)
                else:
                    verdicts['phase_lag'] = ('FAIL ✗', lag)
            
            # Amplitude ratio check (for 5.0 Hz)
            if freq >= 5.0:
                ratio = st['amplitude_ratio']
                if ratio > PASS_CRITERIA['amplitude_ratio_5hz']['excellent']:
                    verdicts['amplitude_ratio'] = ('EXCELLENT ✓', ratio)
                elif ratio > PASS_CRITERIA['amplitude_ratio_5hz']['pass']:
                    verdicts['amplitude_ratio'] = ('PASS ✓', ratio)
                else:
                    verdicts['amplitude_ratio'] = ('FAIL ✗', ratio)
    
    return verdicts

def plot_results(df, results, freq, save_path=None):
    """Create visualization of results"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    
    t = df['time'].values
    f_des = df['f_des'].values
    f_meas = df['f_meas'].values
    f_cmd = df['f_cmd'].values
    
    # Plot 1: Force tracking
    ax1 = axes[0]
    ax1.plot(t, f_des, 'b-', label='Desired', linewidth=1.5)
    ax1.plot(t, f_meas, 'r-', label='Measured', linewidth=1.5, alpha=0.8)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Force (N)')
    ax1.set_title(f'Force Tracking Performance @ {freq} Hz')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Error
    ax2 = axes[1]
    error = f_des - f_meas
    ax2.plot(t, error, 'g-', linewidth=1)
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.5)
    ax2.axhline(y=0.2, color='g', linestyle='--', alpha=0.5, label='Excellent (±0.2N)')
    ax2.axhline(y=-0.2, color='g', linestyle='--', alpha=0.5)
    ax2.axhline(y=1.0, color='orange', linestyle='--', alpha=0.5, label='Pass (±1.0N)')
    ax2.axhline(y=-1.0, color='orange', linestyle='--', alpha=0.5)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Error (N)')
    ax2.set_title('Tracking Error')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Zoomed view (last 4 cycles)
    ax3 = axes[2]
    if freq > 0:
        zoom_start = max(0, t[-1] - 4/freq)
        mask = t >= zoom_start
    else:
        mask = t >= max(0, t[-1] - 5)
    ax3.plot(t[mask], f_des[mask], 'b-', label='Desired', linewidth=2)
    ax3.plot(t[mask], f_meas[mask], 'r-', label='Measured', linewidth=2, alpha=0.8)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Force (N)')
    ax3.set_title('Zoomed View (Last 4 cycles)')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Plot saved to: {save_path}")
    
    plt.show()

def generate_report(results, freq, verdicts):
    """Generate text report"""
    print("\n" + "="*60)
    print(f"FORCE CONTROL VALIDATION REPORT - {freq} Hz")
    print("="*60)
    
    if 'steady_state' in results:
        ss = results['steady_state']
        print("\n📊 Steady-State Performance:")
        print(f"   Mean Error:  {ss['mean_error']:.4f} N")
        print(f"   Std Error:   {ss['std_error']:.4f} N")
        print(f"   Max Error:   {ss['max_error']:.4f} N")
        print(f"   RMS Error:   {ss['rms_error']:.4f} N")
    
    if 'sine_tracking' in results:
        st = results['sine_tracking']
        if 'error' not in st:
            print("\n📈 Sine Tracking Performance:")
            print(f"   Amplitude Cmd:   {st['amplitude_cmd']:.4f} N")
            print(f"   Amplitude Meas:  {st['amplitude_meas']:.4f} N")
            print(f"   Amplitude Ratio: {st['amplitude_ratio']:.4f} ({st['amplitude_attenuation_percent']:.1f}% attenuation)")
            print(f"   Phase Lag:       {st['phase_lag_xcorr_deg']:.2f}°")
        else:
            print(f"\n⚠️ Sine tracking analysis failed: {st['error']}")
    
    print("\n" + "-"*60)
    print("🏆 VERDICT (PI Standards):")
    for metric, (verdict, value) in verdicts.items():
        print(f"   {metric}: {verdict} (value: {value:.4f})")
    print("="*60 + "\n")

def main():
    # Default log file path
    log_path = '/home/andy/franka_ros2_ws/internal_force_log.csv'
    
    if not os.path.exists(log_path):
        print(f"Error: Log file not found at {log_path}")
        print("Please run the force control simulation first.")
        return
    
    # Load data
    print(f"Loading data from: {log_path}")
    df = load_data(log_path)
    print(f"Loaded {len(df)} data points")
    print(f"Time range: {df['time'].min():.2f} - {df['time'].max():.2f} s")
    
    # Detect frequency from data
    # Check if there's significant variation in f_des (sine wave test)
    f_des_range = df['f_des'].max() - df['f_des'].min()
    
    if f_des_range > 0.5:  # Sine wave test
        # Estimate frequency from zero crossings or FFT
        t = df['time'].values
        f_des = df['f_des'].values - np.mean(df['f_des'].values)
        
        # Simple zero-crossing frequency estimation
        zero_crossings = np.where(np.diff(np.signbit(f_des)))[0]
        if len(zero_crossings) >= 2:
            period_samples = np.mean(np.diff(zero_crossings)) * 2
            dt = np.mean(np.diff(t))
            freq = 1.0 / (period_samples * dt)
            print(f"Detected frequency: {freq:.2f} Hz")
        else:
            freq = 0.5  # Default
            print(f"Using default frequency: {freq} Hz")
    else:
        freq = 0.0  # Constant force test
        print("Detected: Constant force test (no sine wave)")
    
    # Run analysis
    results = {}
    results['steady_state'] = analyze_steady_state(df)
    
    if freq > 0:
        results['sine_tracking'] = analyze_sine_tracking(df, freq)
    
    # Check pass/fail
    verdicts = check_pass_fail(results, freq)
    
    # Generate report
    generate_report(results, freq, verdicts)
    
    # Plot results
    plot_path = '/home/andy/franka_ros2_ws/force_control_analysis.png'
    plot_results(df, results, freq, save_path=plot_path)

if __name__ == '__main__':
    main()
