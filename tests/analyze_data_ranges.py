#!/usr/bin/env python3
"""
実データの範囲を詳細に分析するスクリプト
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sys
sys.path.insert(0, '.')

from tire_fitting.data_loader import TireDataLoader


def analyze_drivebrake_data():
    """DriveBrakeデータの分析"""
    print("=" * 70)
    print("DriveBrakeデータの分析")
    print("=" * 70)
    
    loader = TireDataLoader()
    data = loader.load_drivebrake_data(
        'data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv'
    )
    
    print(f"\n総データ数: {len(data)}行")
    
    # 基本統計
    print("\n基本統計:")
    print(f"  SR: [{data['SR'].min():.4f}, {data['SR'].max():.4f}]")
    print(f"  SA: [{data['SA'].min():.4f}, {data['SA'].max():.4f}] rad")
    print(f"  SA: [{np.rad2deg(data['SA'].min()):.2f}, {np.rad2deg(data['SA'].max()):.2f}] deg")
    print(f"  FZ: [{data['FZ'].min():.1f}, {data['FZ'].max():.1f}] N")
    print(f"  FX: [{data['FX'].min():.1f}, {data['FX'].max():.1f}] N")
    print(f"  V: [{data['V'].min():.2f}, {data['V'].max():.2f}] m/s")
    print(f"  V: [{data['V'].min()*3.6:.1f}, {data['V'].max()*3.6:.1f}] kph")
    print(f"  P (temp): [{data['P'].min():.1f}, {data['P'].max():.1f}] °C")
    
    # SA別のデータ分布
    print("\n\nSA（スリップ角）別のデータ分布:")
    sa_thresholds = [0.5, 1.0, 2.0, 5.0, 10.0]
    
    for threshold in sa_thresholds:
        count = len(data[np.abs(data['SA']) < np.deg2rad(threshold)])
        percentage = count / len(data) * 100
        print(f"  |SA| < {threshold:4.1f} deg: {count:6d}行 ({percentage:5.1f}%)")
    
    # SR別のデータ分布
    print("\n\nSR（スリップ比）別のデータ分布:")
    sr_thresholds = [0.01, 0.02, 0.05, 0.10, 0.15]
    
    for threshold in sr_thresholds:
        count = len(data[np.abs(data['SR']) < threshold])
        percentage = count / len(data) * 100
        print(f"  |SR| < {threshold:.2f}: {count:6d}行 ({percentage:5.1f}%)")
    
    # SA vs SR の関係
    print("\n\nSA vs SR の関係:")
    print("  SA < 0.5 deg かつ SR > 0.05:")
    count = len(data[(np.abs(data['SA']) < np.deg2rad(0.5)) & (np.abs(data['SR']) > 0.05)])
    print(f"    {count}行 ({count/len(data)*100:.1f}%)")
    
    print("  SA < 5.0 deg かつ SR > 0.05:")
    count = len(data[(np.abs(data['SA']) < np.deg2rad(5.0)) & (np.abs(data['SR']) > 0.05)])
    print(f"    {count}行 ({count/len(data)*100:.1f}%)")
    
    # 可視化
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # プロット1: SA分布
    axes[0, 0].hist(np.rad2deg(data['SA']), bins=100, alpha=0.7, edgecolor='black')
    axes[0, 0].axvline(x=0.5, color='r', linestyle='--', label='Current threshold (0.5 deg)')
    axes[0, 0].axvline(x=5.0, color='g', linestyle='--', label='Proposed threshold (5.0 deg)')
    axes[0, 0].axvline(x=-0.5, color='r', linestyle='--')
    axes[0, 0].axvline(x=-5.0, color='g', linestyle='--')
    axes[0, 0].set_xlabel('Slip Angle [deg]')
    axes[0, 0].set_ylabel('Frequency')
    axes[0, 0].set_title('Slip Angle Distribution (DriveBrake)')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    # プロット2: SR分布
    axes[0, 1].hist(data['SR'], bins=100, alpha=0.7, edgecolor='black')
    axes[0, 1].axvline(x=0.01, color='r', linestyle='--', label='Current range (±0.01)')
    axes[0, 1].axvline(x=-0.01, color='r', linestyle='--')
    axes[0, 1].set_xlabel('Slip Ratio [-]')
    axes[0, 1].set_ylabel('Frequency')
    axes[0, 1].set_title('Slip Ratio Distribution (DriveBrake)')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # プロット3: SA vs SR
    axes[1, 0].scatter(np.rad2deg(data['SA']), data['SR'], alpha=0.1, s=1)
    axes[1, 0].axhline(y=0.05, color='orange', linestyle='--', label='SR = 0.05')
    axes[1, 0].axhline(y=-0.05, color='orange', linestyle='--')
    axes[1, 0].axvline(x=0.5, color='r', linestyle='--', label='SA = 0.5 deg')
    axes[1, 0].axvline(x=-0.5, color='r', linestyle='--')
    axes[1, 0].axvline(x=5.0, color='g', linestyle='--', label='SA = 5.0 deg')
    axes[1, 0].axvline(x=-5.0, color='g', linestyle='--')
    axes[1, 0].set_xlabel('Slip Angle [deg]')
    axes[1, 0].set_ylabel('Slip Ratio [-]')
    axes[1, 0].set_title('SA vs SR Relationship')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].set_xlim(-15, 15)
    axes[1, 0].set_ylim(-0.3, 0.2)
    
    # プロット4: |SA| vs |SR|
    axes[1, 1].scatter(np.abs(np.rad2deg(data['SA'])), np.abs(data['SR']), alpha=0.1, s=1)
    axes[1, 1].axvline(x=0.5, color='r', linestyle='--', linewidth=2, label='Current (0.5 deg)')
    axes[1, 1].axvline(x=5.0, color='g', linestyle='--', linewidth=2, label='Proposed (5.0 deg)')
    axes[1, 1].set_xlabel('|Slip Angle| [deg]')
    axes[1, 1].set_ylabel('|Slip Ratio| [-]')
    axes[1, 1].set_title('|SA| vs |SR| Relationship')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].set_xlim(0, 15)
    axes[1, 1].set_ylim(0, 0.3)
    
    plt.tight_layout()
    plt.savefig('output/data_range_analysis_drivebrake.png', dpi=150, bbox_inches='tight')
    print(f"\nグラフを保存: output/data_range_analysis_drivebrake.png")
    
    return data


def analyze_cornering_data():
    """Corneringデータの分析"""
    print("\n" + "=" * 70)
    print("Corneringデータの分析")
    print("=" * 70)
    
    loader = TireDataLoader()
    data = loader.load_cornering_data(
        'data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv'
    )
    
    print(f"\n総データ数: {len(data)}行")
    
    # 基本統計
    print("\n基本統計:")
    print(f"  SA: [{data['SA'].min():.4f}, {data['SA'].max():.4f}] rad")
    print(f"  SA: [{np.rad2deg(data['SA'].min()):.2f}, {np.rad2deg(data['SA'].max()):.2f}] deg")
    
    if 'SR' in data.columns:
        print(f"  SR: [{data['SR'].min():.4f}, {data['SR'].max():.4f}]")
    
    print(f"  FZ: [{data['FZ'].min():.1f}, {data['FZ'].max():.1f}] N")
    print(f"  FY: [{data['FY'].min():.1f}, {data['FY'].max():.1f}] N")
    
    # SR別のデータ分布（Corneringデータにある場合）
    if 'SR' in data.columns:
        print("\n\nSR（スリップ比）別のデータ分布:")
        sr_thresholds = [0.01, 0.02, 0.05, 0.10, 0.15]
        
        for threshold in sr_thresholds:
            count = len(data[np.abs(data['SR']) < threshold])
            percentage = count / len(data) * 100
            print(f"  |SR| < {threshold:.2f}: {count:6d}行 ({percentage:5.1f}%)")
    
    return data


def main():
    """メイン実行"""
    print("\n" + "=" * 70)
    print("実データ範囲の詳細分析")
    print("=" * 70)
    print()
    
    # DriveBrakeデータ分析
    drivebrake_data = analyze_drivebrake_data()
    
    # Corneringデータ分析
    cornering_data = analyze_cornering_data()
    
    # 推奨フィルタリング条件
    print("\n" + "=" * 70)
    print("推奨フィルタリング条件")
    print("=" * 70)
    
    print("\n現在の条件（v3/v4）:")
    print("  max_sa_for_longitudinal = 0.5 deg")
    print("  max_sr_for_lateral = 0.01")
    print("  結果: 900行 (1.2%)")
    
    print("\n推奨条件（v5）:")
    print("  max_sa_for_longitudinal = 5.0 deg  (10倍に緩和)")
    print("  max_sr_for_lateral = 0.05          (5倍に緩和)")
    
    # v5での期待データ数を計算
    drivebrake_v5 = drivebrake_data[
        (drivebrake_data['V'] > 10.0/3.6) &
        (drivebrake_data['FZ'] > 200.0) &
        (drivebrake_data['P'] > 20.0) &
        (drivebrake_data['P'] < 100.0) &
        (np.abs(drivebrake_data['SA']) < np.deg2rad(5.0))
    ]
    
    print(f"  期待データ数（DriveBrake）: {len(drivebrake_v5)}行 ({len(drivebrake_v5)/len(drivebrake_data)*100:.1f}%)")
    print(f"  SR範囲: [{drivebrake_v5['SR'].min():.4f}, {drivebrake_v5['SR'].max():.4f}]")
    
    print("\n期待される効果:")
    print(f"  データ量: 900 → {len(drivebrake_v5)}行 ({len(drivebrake_v5)/900:.1f}倍)")
    print(f"  SR範囲: ±0.01 → ±{max(abs(drivebrake_v5['SR'].min()), abs(drivebrake_v5['SR'].max())):.2f}")
    print("  高スリップ領域のデータを含む")
    print("  mu_peak推定が可能になる")
    
    print("\n" + "=" * 70)


if __name__ == '__main__':
    main()
