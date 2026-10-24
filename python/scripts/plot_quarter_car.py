#!/usr/bin/env python3
"""
Quarter Car シミュレーション結果の可視化
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


def plot_quarter_car(df: pd.DataFrame, output_dir: Path = None):
    """Quarter Car結果をプロット"""
    
    # スタイル設定
    plt.style.use('seaborn-v0_8-whitegrid')
    plt.rcParams.update({
        'font.size': 10,
        'axes.labelsize': 12,
        'axes.titlesize': 14,
        'figure.figsize': (14, 10),
        'figure.dpi': 100,
    })
    
    t = df['time'].values
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # 1. 変位
    ax1 = axes[0, 0]
    ax1.plot(t, df['z_s'], 'b-', linewidth=1.5, label='Sprung (z_s)')
    ax1.plot(t, df['z_u'], 'r-', linewidth=1.5, label='Unsprung (z_u)')
    ax1.plot(t, df['z_road'], 'k--', linewidth=1, label='Road input')
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('Displacement [mm]')
    ax1.set_title('Suspension Displacement')
    ax1.legend(loc='best')
    ax1.grid(True, alpha=0.3)
    
    # 2. 速度
    ax2 = axes[0, 1]
    ax2.plot(t, df['dz_s'], 'b-', linewidth=1.5, label='Sprung (dz_s)')
    ax2.plot(t, df['dz_u'], 'r-', linewidth=1.5, label='Unsprung (dz_u)')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Velocity [mm/s]')
    ax2.set_title('Suspension Velocity')
    ax2.legend(loc='best')
    ax2.grid(True, alpha=0.3)
    
    # 3. 力
    ax3 = axes[1, 0]
    ax3.plot(t, df['F_spring'], 'g-', linewidth=1.5, label='Spring')
    ax3.plot(t, df['F_damper'], 'm-', linewidth=1.5, label='Damper (Bouc-Wen)')
    ax3.plot(t, df['F_tire'], 'c-', linewidth=1.5, label='Tire')
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Force [N]')
    ax3.set_title('Suspension Forces')
    ax3.legend(loc='best')
    ax3.grid(True, alpha=0.3)
    
    # 4. ダンパーF-V特性（ヒステリシス）
    ax4 = axes[1, 1]
    v_rel = df['dz_s'].values - df['dz_u'].values  # 相対速度
    F_d = df['F_damper'].values
    
    # 時間で色分け
    colors = plt.cm.viridis(np.linspace(0, 1, len(t)))
    for i in range(len(t) - 1):
        ax4.plot(v_rel[i:i+2], F_d[i:i+2], color=colors[i], linewidth=0.5)
    
    sm = plt.cm.ScalarMappable(cmap='viridis', norm=plt.Normalize(vmin=t[0], vmax=t[-1]))
    plt.colorbar(sm, ax=ax4, label='Time [s]')
    ax4.set_xlabel('Relative Velocity [mm/s]')
    ax4.set_ylabel('Damper Force [N]')
    ax4.set_title('Damper Hysteresis (F-V Loop)')
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.suptitle('Quarter Car Simulation with Bouc-Wen Damper', y=1.02, fontsize=16)
    
    # 保存
    if output_dir:
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        output_path = output_dir / 'quarter_car_plot.png'
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"Saved plot to {output_path}")
    
    plt.show()
    
    return fig


def print_summary(df: pd.DataFrame):
    """結果のサマリーを表示"""
    print("\n" + "="*50)
    print("SIMULATION SUMMARY")
    print("="*50)
    print(f"Duration: {df['time'].iloc[-1]:.2f} s")
    print(f"Data points: {len(df)}")
    print(f"\nPeak values:")
    print(f"  Max z_s: {df['z_s'].max():.2f} mm")
    print(f"  Max z_u: {df['z_u'].max():.2f} mm")
    print(f"  Max |dz_s|: {df['dz_s'].abs().max():.1f} mm/s")
    print(f"  Max |F_damper|: {df['F_damper'].abs().max():.0f} N")
    print(f"\nFinal state:")
    print(f"  z_s: {df['z_s'].iloc[-1]:.3f} mm")
    print(f"  z_u: {df['z_u'].iloc[-1]:.3f} mm")
    print("="*50)


def main():
    # デフォルトパス
    if len(sys.argv) >= 2:
        csv_path = sys.argv[1]
    else:
        csv_path = "/workspace/build/quarter_car_result.csv"
    
    output_dir = Path(sys.argv[2]) if len(sys.argv) >= 3 else Path("/workspace/output")
    
    # データ読み込み
    df = load_data(csv_path)
    
    # サマリー表示
    print_summary(df)
    
    # プロット
    plot_quarter_car(df, output_dir)


if __name__ == "__main__":
    main()
