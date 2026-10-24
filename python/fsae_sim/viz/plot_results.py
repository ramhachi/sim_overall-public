"""
シミュレーション結果の可視化
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
from typing import Optional, List, Dict


def setup_plot_style():
    """プロットスタイル設定"""
    plt.style.use('seaborn-v0_8-whitegrid')
    plt.rcParams.update({
        'font.size': 10,
        'axes.labelsize': 12,
        'axes.titlesize': 14,
        'legend.fontsize': 10,
        'figure.figsize': (12, 8),
        'figure.dpi': 100,
    })


def plot_vehicle_response(
    result_file: Path,
    output_dir: Optional[Path] = None,
    show: bool = True
):
    """車両応答のプロット"""
    setup_plot_style()
    
    # データ読み込み
    df = pd.read_csv(result_file)
    t = df['time'].values
    
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    
    # 位置
    if 'x' in df.columns and 'y' in df.columns:
        axes[0, 0].plot(df['x'], df['y'], 'b-', linewidth=1.5)
        axes[0, 0].set_xlabel('X [m]')
        axes[0, 0].set_ylabel('Y [m]')
        axes[0, 0].set_title('Vehicle Trajectory')
        axes[0, 0].axis('equal')
    
    # 速度
    if 'vx' in df.columns and 'vy' in df.columns:
        axes[0, 1].plot(t, df['vx'], 'b-', label='Vx')
        axes[0, 1].plot(t, df['vy'], 'r-', label='Vy')
        axes[0, 1].set_xlabel('Time [s]')
        axes[0, 1].set_ylabel('Velocity [m/s]')
        axes[0, 1].set_title('Body Velocities')
        axes[0, 1].legend()
    
    # オイラー角
    if 'phi' in df.columns:
        axes[1, 0].plot(t, np.degrees(df['phi']), 'b-', label='Roll')
        if 'theta' in df.columns:
            axes[1, 0].plot(t, np.degrees(df['theta']), 'r-', label='Pitch')
        if 'psi' in df.columns:
            axes[1, 0].plot(t, np.degrees(df['psi']), 'g-', label='Yaw')
        axes[1, 0].set_xlabel('Time [s]')
        axes[1, 0].set_ylabel('Angle [deg]')
        axes[1, 0].set_title('Body Attitude')
        axes[1, 0].legend()
    
    # 角速度
    if 'r' in df.columns:
        axes[1, 1].plot(t, np.degrees(df['r']), 'b-')
        axes[1, 1].set_xlabel('Time [s]')
        axes[1, 1].set_ylabel('Yaw Rate [deg/s]')
        axes[1, 1].set_title('Yaw Rate')
    
    # タイヤ力（存在する場合）
    tire_cols = [c for c in df.columns if c.startswith('Fy_')]
    if tire_cols:
        for col in tire_cols:
            axes[2, 0].plot(t, df[col], label=col)
        axes[2, 0].set_xlabel('Time [s]')
        axes[2, 0].set_ylabel('Lateral Force [N]')
        axes[2, 0].set_title('Tire Lateral Forces')
        axes[2, 0].legend()
    
    # タイヤ温度（存在する場合）
    temp_cols = [c for c in df.columns if c.startswith('T_')]
    if temp_cols:
        for col in temp_cols:
            axes[2, 1].plot(t, df[col], label=col)
        axes[2, 1].set_xlabel('Time [s]')
        axes[2, 1].set_ylabel('Temperature [°C]')
        axes[2, 1].set_title('Tire Temperatures')
        axes[2, 1].legend()
    
    plt.tight_layout()
    
    if output_dir:
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        plt.savefig(output_dir / 'vehicle_response.png', dpi=150)
        print(f"Saved: {output_dir / 'vehicle_response.png'}")
    
    if show:
        plt.show()
    
    return fig


def plot_comparison(
    matlab_file: Path,
    cpp_file: Path,
    output_dir: Optional[Path] = None,
    show: bool = True
):
    """MATLABとC++結果の比較プロット"""
    setup_plot_style()
    
    df_matlab = pd.read_csv(matlab_file, header=None)
    df_cpp = pd.read_csv(cpp_file, header=None)
    
    t_matlab = df_matlab.iloc[:, 0].values
    t_cpp = df_cpp.iloc[:, 0].values
    
    n_states = min(df_matlab.shape[1] - 1, df_cpp.shape[1] - 1, 4)
    state_names = ['z_s', 'z_u', 'dz_s', 'dz_u']
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    axes = axes.flatten()
    
    for i in range(n_states):
        x_matlab = df_matlab.iloc[:, i + 1].values
        x_cpp = df_cpp.iloc[:, i + 1].values
        
        axes[i].plot(t_matlab, x_matlab, 'b-', linewidth=1.5, label='MATLAB')
        axes[i].plot(t_cpp, x_cpp, 'r--', linewidth=1.5, label='C++')
        axes[i].set_xlabel('Time [s]')
        axes[i].set_ylabel(state_names[i])
        axes[i].set_title(f'{state_names[i]} Comparison')
        axes[i].legend()
    
    plt.tight_layout()
    plt.suptitle('MATLAB vs C++ Verification', y=1.02)
    
    if output_dir:
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        plt.savefig(output_dir / 'comparison.png', dpi=150)
        print(f"Saved: {output_dir / 'comparison.png'}")
    
    if show:
        plt.show()
    
    return fig


if __name__ == "__main__":
    import sys
    if len(sys.argv) >= 2:
        result_file = Path(sys.argv[1])
        output_dir = Path(sys.argv[2]) if len(sys.argv) >= 3 else None
        plot_vehicle_response(result_file, output_dir)
    else:
        print("Usage: python plot_results.py <result_file.csv> [output_dir]")
