#!/usr/bin/env python3
"""
MATLAB vs C++ 比較検証スクリプト
両者の結果を読み込み、誤差を計算・可視化
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import sys


def load_results(cpp_path: str, matlab_path: str):
    """両方の結果を読み込み"""
    cpp_df = pd.read_csv(cpp_path)
    matlab_df = pd.read_csv(matlab_path)
    
    print(f"Loaded C++ result: {len(cpp_df)} points")
    print(f"Loaded MATLAB result: {len(matlab_df)} points")
    
    return cpp_df, matlab_df


def compute_errors(cpp_df: pd.DataFrame, matlab_df: pd.DataFrame):
    """誤差を計算"""
    # 時間軸を合わせる（同じ長さを仮定）
    n = min(len(cpp_df), len(matlab_df))
    
    errors = {}
    columns = ['z_s', 'z_u', 'dz_s', 'dz_u', 'F_spring', 'F_damper', 'F_tire']
    
    for col in columns:
        if col in cpp_df.columns and col in matlab_df.columns:
            diff = cpp_df[col].values[:n] - matlab_df[col].values[:n]
            errors[col] = {
                'max_abs': np.max(np.abs(diff)),
                'rmse': np.sqrt(np.mean(diff**2)),
                'mean': np.mean(diff),
                'diff': diff
            }
    
    return errors, n


def print_error_summary(errors: dict):
    """誤差サマリーを表示"""
    print("\n" + "="*60)
    print("VERIFICATION RESULTS: MATLAB vs C++")
    print("="*60)
    
    print(f"\n{'Variable':<12} {'Max Abs Error':<15} {'RMSE':<15} {'Mean Error':<15}")
    print("-"*60)
    
    all_pass = True
    for var, err in errors.items():
        max_err = err['max_abs']
        rmse = err['rmse']
        mean = err['mean']
        
        # 判定（閾値は変数によって調整）
        if 'F_' in var:
            threshold = 1.0  # 力は1N以下
        else:
            threshold = 0.01  # 変位・速度は0.01mm以下
        
        status = "✓" if max_err < threshold else "✗"
        if max_err >= threshold:
            all_pass = False
        
        print(f"{var:<12} {max_err:<15.6f} {rmse:<15.6f} {mean:<15.6f} {status}")
    
    print("-"*60)
    if all_pass:
        print("RESULT: PASS ✓ - All errors within tolerance")
    else:
        print("RESULT: FAIL ✗ - Some errors exceed tolerance")
    print("="*60)
    
    return all_pass


def plot_comparison(cpp_df: pd.DataFrame, matlab_df: pd.DataFrame, 
                    errors: dict, n: int, output_dir: Path):
    """比較プロットを作成"""
    
    plt.style.use('seaborn-v0_8-whitegrid')
    fig, axes = plt.subplots(3, 2, figsize=(14, 12))
    
    t_cpp = cpp_df['time'].values[:n]
    t_matlab = matlab_df['time'].values[:n]
    
    # z_s 比較
    ax = axes[0, 0]
    ax.plot(t_cpp, cpp_df['z_s'].values[:n], 'b-', linewidth=1.5, label='C++')
    ax.plot(t_matlab, matlab_df['z_s'].values[:n], 'r--', linewidth=1.5, label='MATLAB')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('z_s [mm]')
    ax.set_title('Sprung Mass Displacement')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # z_u 比較
    ax = axes[0, 1]
    ax.plot(t_cpp, cpp_df['z_u'].values[:n], 'b-', linewidth=1.5, label='C++')
    ax.plot(t_matlab, matlab_df['z_u'].values[:n], 'r--', linewidth=1.5, label='MATLAB')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('z_u [mm]')
    ax.set_title('Unsprung Mass Displacement')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # F_damper 比較
    ax = axes[1, 0]
    ax.plot(t_cpp, cpp_df['F_damper'].values[:n], 'b-', linewidth=1.5, label='C++')
    ax.plot(t_matlab, matlab_df['F_damper'].values[:n], 'r--', linewidth=1.5, label='MATLAB')
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('F_damper [N]')
    ax.set_title('Damper Force (Bouc-Wen)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 誤差（z_s）
    ax = axes[1, 1]
    ax.plot(t_cpp, errors['z_s']['diff'], 'k-', linewidth=1)
    ax.axhline(y=0, color='r', linestyle='--', linewidth=0.5)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Error [mm]')
    ax.set_title(f'z_s Error (RMSE={errors["z_s"]["rmse"]:.2e})')
    ax.grid(True, alpha=0.3)
    
    # 誤差（F_damper）
    ax = axes[2, 0]
    ax.plot(t_cpp, errors['F_damper']['diff'], 'k-', linewidth=1)
    ax.axhline(y=0, color='r', linestyle='--', linewidth=0.5)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Error [N]')
    ax.set_title(f'F_damper Error (RMSE={errors["F_damper"]["rmse"]:.2e})')
    ax.grid(True, alpha=0.3)
    
    # F-Vループ比較
    ax = axes[2, 1]
    v_rel_cpp = cpp_df['dz_s'].values[:n] - cpp_df['dz_u'].values[:n]
    v_rel_matlab = matlab_df['dz_s'].values[:n] - matlab_df['dz_u'].values[:n]
    ax.plot(v_rel_cpp, cpp_df['F_damper'].values[:n], 'b-', linewidth=1, alpha=0.7, label='C++')
    ax.plot(v_rel_matlab, matlab_df['F_damper'].values[:n], 'r--', linewidth=1, alpha=0.7, label='MATLAB')
    ax.set_xlabel('Relative Velocity [mm/s]')
    ax.set_ylabel('Damper Force [N]')
    ax.set_title('Damper Hysteresis Comparison')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.suptitle('MATLAB vs C++ Verification', y=1.02, fontsize=16)
    
    # 保存
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / 'verification_comparison.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\nSaved comparison plot to {output_path}")
    
    plt.show()


def main():
    # デフォルトパス
    if len(sys.argv) >= 3:
        cpp_path = sys.argv[1]
        matlab_path = sys.argv[2]
    else:
        cpp_path = "/workspace/build/quarter_car_result.csv"
        matlab_path = "/workspace/output/quarter_car_matlab.csv"
    
    output_dir = Path(sys.argv[3]) if len(sys.argv) >= 4 else Path("/workspace/output")
    
    # データ読み込み
    cpp_df, matlab_df = load_results(cpp_path, matlab_path)
    
    # 誤差計算
    errors, n = compute_errors(cpp_df, matlab_df)
    
    # サマリー表示
    passed = print_error_summary(errors)
    
    # プロット
    plot_comparison(cpp_df, matlab_df, errors, n, output_dir)
    
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
