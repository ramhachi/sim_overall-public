#!/usr/bin/env python3
"""
14DOFシミュレーション結果の可視化
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import sys


def load_data(csv_path: str) -> pd.DataFrame:
    """CSVデータを読み込み"""
    df = pd.read_csv(csv_path)
    print(f"Loaded {len(df)} data points from {csv_path}")
    return df


def plot_step_steer(df: pd.DataFrame, output_dir: Path = None):
    """ステップステア試験結果をプロット"""
    
    plt.style.use('seaborn-v0_8-whitegrid')
    fig, axes = plt.subplots(3, 2, figsize=(14, 12))
    
    t = df['time'].values
    
    # 1. 軌跡
    ax = axes[0, 0]
    ax.plot(df['x'], df['y'], 'b-', linewidth=1.5)
    ax.plot(df['x'].iloc[0], df['y'].iloc[0], 'go', markersize=10, label='Start')
    ax.plot(df['x'].iloc[-1], df['y'].iloc[-1], 'ro', markersize=10, label='End')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title('Vehicle Trajectory')
    ax.axis('equal')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 2. ヨー角
    ax = axes[0, 1]
    ax.plot(t, df['psi'], 'b-', linewidth=1.5)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Yaw Angle [deg]')
    ax.set_title('Yaw Angle')
    ax.grid(True, alpha=0.3)
    
    # 3. 速度
    ax = axes[1, 0]
    ax.plot(t, df['vx'], 'b-', linewidth=1.5, label='Vx')
    ax.plot(t, df['vy'], 'r-', linewidth=1.5, label='Vy')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Velocity [m/s]')
    ax.set_title('Velocity Components')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 4. ヨーレート
    ax = axes[1, 1]
    ax.plot(t, df['r'], 'b-', linewidth=1.5)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Yaw Rate [deg/s]')
    ax.set_title('Yaw Rate')
    ax.grid(True, alpha=0.3)
    
    # 5. 横加速度
    ax = axes[2, 0]
    ax.plot(t, df['ay'] / 9.81, 'b-', linewidth=1.5)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Lateral Acceleration [G]')
    ax.set_title('Lateral Acceleration')
    ax.grid(True, alpha=0.3)
    
    # 6. 入力
    ax = axes[2, 1]
    ax.plot(t, df['steering'], 'b-', linewidth=1.5, label='Steering')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Steering [deg]')
    ax.set_title('Driver Input')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.suptitle('Step Steer Test - 14DOF Vehicle Model', y=1.02, fontsize=16)
    
    if output_dir:
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        output_path = output_dir / 'step_steer_plot.png'
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"Saved plot to {output_path}")
    
    plt.show()
    return fig


def plot_steady_circle(df: pd.DataFrame, output_dir: Path = None):
    """定常円旋回試験結果をプロット"""
    
    plt.style.use('seaborn-v0_8-whitegrid')
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    t = df['time'].values
    
    # 1. 軌跡
    ax = axes[0, 0]
    ax.plot(df['x'], df['y'], 'b-', linewidth=1.5)
    ax.plot(df['x'].iloc[0], df['y'].iloc[0], 'go', markersize=10, label='Start')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title('Vehicle Trajectory (Steady Circle)')
    ax.axis('equal')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 2. 速度
    ax = axes[0, 1]
    speed = np.sqrt(df['vx']**2 + df['vy']**2)
    ax.plot(t, speed, 'b-', linewidth=1.5)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Speed [m/s]')
    ax.set_title('Vehicle Speed')
    ax.grid(True, alpha=0.3)
    
    # 3. ヨーレート
    ax = axes[1, 0]
    ax.plot(t, df['r'], 'b-', linewidth=1.5)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Yaw Rate [deg/s]')
    ax.set_title('Yaw Rate')
    ax.grid(True, alpha=0.3)
    
    # 4. 横加速度
    ax = axes[1, 1]
    ax.plot(t, df['ay'] / 9.81, 'b-', linewidth=1.5)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Lateral Acceleration [G]')
    ax.set_title('Lateral Acceleration')
    ax.axhline(y=1.0, color='r', linestyle='--', linewidth=0.5, label='1G')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.suptitle('Steady State Circle Test - 14DOF Vehicle Model', y=1.02, fontsize=16)
    
    if output_dir:
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        output_path = output_dir / 'steady_circle_plot.png'
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"Saved plot to {output_path}")
    
    plt.show()
    return fig


def print_summary(df: pd.DataFrame, test_type: str):
    """結果サマリーを表示"""
    print("\n" + "="*50)
    print(f"14DOF SIMULATION SUMMARY: {test_type}")
    print("="*50)
    print(f"Duration: {df['time'].iloc[-1]:.2f} s")
    print(f"Data points: {len(df)}")
    print(f"\nFinal state:")
    print(f"  Position: ({df['x'].iloc[-1]:.1f}, {df['y'].iloc[-1]:.1f}) m")
    print(f"  Yaw: {df['psi'].iloc[-1]:.1f} deg")
    print(f"  Speed: {np.sqrt(df['vx'].iloc[-1]**2 + df['vy'].iloc[-1]**2):.1f} m/s")
    print(f"\nPeak values:")
    print(f"  Max |Ay|: {df['ay'].abs().max() / 9.81:.2f} G")
    print(f"  Max |Yaw rate|: {df['r'].abs().max():.1f} deg/s")
    print("="*50)


def main():
    # 引数解析
    if len(sys.argv) < 2:
        csv_path = "/workspace/build/step_steer_result.csv"
        test_type = "step_steer"
    else:
        csv_path = sys.argv[1]
        test_type = "step_steer" if "step_steer" in csv_path else "circle"
    
    output_dir = Path(sys.argv[2]) if len(sys.argv) >= 3 else Path("/workspace/output")
    
    # データ読み込み
    df = load_data(csv_path)
    
    # サマリー表示
    print_summary(df, test_type)
    
    # プロット
    if test_type == "step_steer":
        plot_step_steer(df, output_dir)
    else:
        plot_steady_circle(df, output_dir)


if __name__ == "__main__":
    main()
