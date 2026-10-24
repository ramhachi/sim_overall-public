#!/usr/bin/env python3
"""
スリップ角（SA）の異常値を詳細に分析するスクリプト

目的:
1. SA = -344 deg などの異常値の原因を特定
2. データの単位や変換の問題を確認
3. 正しい値への修正方法を提案
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sys
sys.path.insert(0, '.')

from tire_fitting.data_loader import TireDataLoader


def analyze_sa_distribution():
    """SAの分布を詳細に分析"""
    print("=" * 70)
    print("スリップ角（SA）の異常値分析")
    print("=" * 70)
    
    loader = TireDataLoader()
    data = loader.load_drivebrake_data(
        'data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv'
    )
    
    print(f"\n総データ数: {len(data)}行")
    
    # SAの基本統計
    print("\n基本統計（ラジアン）:")
    print(f"  最小値: {data['SA'].min():.4f} rad")
    print(f"  最大値: {data['SA'].max():.4f} rad")
    print(f"  平均値: {data['SA'].mean():.4f} rad")
    print(f"  中央値: {data['SA'].median():.4f} rad")
    print(f"  標準偏差: {data['SA'].std():.4f} rad")
    
    print("\n基本統計（度）:")
    print(f"  最小値: {np.rad2deg(data['SA'].min()):.2f} deg")
    print(f"  最大値: {np.rad2deg(data['SA'].max()):.2f} deg")
    print(f"  平均値: {np.rad2deg(data['SA'].mean()):.2f} deg")
    print(f"  中央値: {np.rad2deg(data['SA'].median()):.2f} deg")
    print(f"  標準偏差: {np.rad2deg(data['SA'].std()):.2f} deg")
    
    # パーセンタイル
    print("\nパーセンタイル（度）:")
    percentiles = [1, 5, 10, 25, 50, 75, 90, 95, 99]
    for p in percentiles:
        val = np.percentile(data['SA'], p)
        print(f"  {p:2d}%: {np.rad2deg(val):8.2f} deg")
    
    # 異常値の特定
    print("\n異常値の分析:")
    
    # |SA| > 10 deg
    anomaly_10 = data[np.abs(data['SA']) > np.deg2rad(10)]
    print(f"  |SA| > 10 deg: {len(anomaly_10)}行 ({len(anomaly_10)/len(data)*100:.1f}%)")
    
    # |SA| > 45 deg
    anomaly_45 = data[np.abs(data['SA']) > np.deg2rad(45)]
    print(f"  |SA| > 45 deg: {len(anomaly_45)}行 ({len(anomaly_45)/len(data)*100:.1f}%)")
    
    # |SA| > 90 deg
    anomaly_90 = data[np.abs(data['SA']) > np.deg2rad(90)]
    print(f"  |SA| > 90 deg: {len(anomaly_90)}行 ({len(anomaly_90)/len(data)*100:.1f}%)")
    
    # |SA| > 180 deg
    anomaly_180 = data[np.abs(data['SA']) > np.deg2rad(180)]
    print(f"  |SA| > 180 deg: {len(anomaly_180)}行 ({len(anomaly_180)/len(data)*100:.1f}%)")
    
    # 最も異常な値を表示
    print("\n最も異常な値（上位10件）:")
    extreme_values = data.nsmallest(10, 'SA')[['SA', 'SR', 'FX', 'FZ']]
    for idx, row in extreme_values.iterrows():
        print(f"  SA={np.rad2deg(row['SA']):8.2f} deg, SR={row['SR']:7.4f}, FX={row['FX']:8.1f}N, FZ={row['FZ']:7.1f}N")
    
    # SAの値の分布を確認
    print("\nSAの値の範囲別分布:")
    ranges = [
        (-np.inf, np.deg2rad(-180)),
        (np.deg2rad(-180), np.deg2rad(-90)),
        (np.deg2rad(-90), np.deg2rad(-45)),
        (np.deg2rad(-45), np.deg2rad(-10)),
        (np.deg2rad(-10), np.deg2rad(-5)),
        (np.deg2rad(-5), np.deg2rad(5)),
        (np.deg2rad(5), np.deg2rad(10)),
        (np.deg2rad(10), np.deg2rad(45)),
        (np.deg2rad(45), np.deg2rad(90)),
        (np.deg2rad(90), np.deg2rad(180)),
        (np.deg2rad(180), np.inf),
    ]
    
    for lower, upper in ranges:
        count = len(data[(data['SA'] >= lower) & (data['SA'] < upper)])
        percentage = count / len(data) * 100
        print(f"  [{np.rad2deg(lower):7.1f}, {np.rad2deg(upper):7.1f}) deg: {count:6d}行 ({percentage:5.1f}%)")
    
    # 可視化
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    
    # プロット1: SAのヒストグラム（全範囲）
    axes[0, 0].hist(np.rad2deg(data['SA']), bins=100, alpha=0.7, edgecolor='black')
    axes[0, 0].set_xlabel('Slip Angle [deg]')
    axes[0, 0].set_ylabel('Frequency')
    axes[0, 0].set_title('SA Distribution (Full Range)')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].axvline(x=-180, color='r', linestyle='--', label='±180 deg')
    axes[0, 0].axvline(x=180, color='r', linestyle='--')
    axes[0, 0].legend()
    
    # プロット2: SAのヒストグラム（-45 to 45 deg）
    sa_filtered = data[(data['SA'] >= np.deg2rad(-45)) & (data['SA'] <= np.deg2rad(45))]
    axes[0, 1].hist(np.rad2deg(sa_filtered['SA']), bins=100, alpha=0.7, edgecolor='black')
    axes[0, 1].set_xlabel('Slip Angle [deg]')
    axes[0, 1].set_ylabel('Frequency')
    axes[0, 1].set_title('SA Distribution (-45 to 45 deg)')
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].axvline(x=-5, color='g', linestyle='--', label='±5 deg')
    axes[0, 1].axvline(x=5, color='g', linestyle='--')
    axes[0, 1].legend()
    
    # プロット3: SAのヒストグラム（-10 to 10 deg）
    sa_filtered2 = data[(data['SA'] >= np.deg2rad(-10)) & (data['SA'] <= np.deg2rad(10))]
    axes[0, 2].hist(np.rad2deg(sa_filtered2['SA']), bins=100, alpha=0.7, edgecolor='black')
    axes[0, 2].set_xlabel('Slip Angle [deg]')
    axes[0, 2].set_ylabel('Frequency')
    axes[0, 2].set_title('SA Distribution (-10 to 10 deg)')
    axes[0, 2].grid(True, alpha=0.3)
    
    # プロット4: SA vs SR
    axes[1, 0].scatter(np.rad2deg(data['SA']), data['SR'], alpha=0.1, s=1)
    axes[1, 0].set_xlabel('Slip Angle [deg]')
    axes[1, 0].set_ylabel('Slip Ratio [-]')
    axes[1, 0].set_title('SA vs SR')
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].axvline(x=-180, color='r', linestyle='--', alpha=0.5)
    axes[1, 0].axvline(x=180, color='r', linestyle='--', alpha=0.5)
    
    # プロット5: SA vs FX
    axes[1, 1].scatter(np.rad2deg(data['SA']), data['FX'], alpha=0.1, s=1)
    axes[1, 1].set_xlabel('Slip Angle [deg]')
    axes[1, 1].set_ylabel('Longitudinal Force [N]')
    axes[1, 1].set_title('SA vs FX')
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].axvline(x=-180, color='r', linestyle='--', alpha=0.5)
    axes[1, 1].axvline(x=180, color='r', linestyle='--', alpha=0.5)
    
    # プロット6: 累積分布
    sorted_sa = np.sort(np.rad2deg(data['SA']))
    cumulative = np.arange(1, len(sorted_sa) + 1) / len(sorted_sa) * 100
    axes[1, 2].plot(sorted_sa, cumulative, linewidth=2)
    axes[1, 2].set_xlabel('Slip Angle [deg]')
    axes[1, 2].set_ylabel('Cumulative Percentage [%]')
    axes[1, 2].set_title('Cumulative Distribution')
    axes[1, 2].grid(True, alpha=0.3)
    axes[1, 2].axvline(x=-180, color='r', linestyle='--', label='±180 deg')
    axes[1, 2].axvline(x=180, color='r', linestyle='--')
    axes[1, 2].axvline(x=-5, color='g', linestyle='--', label='±5 deg')
    axes[1, 2].axvline(x=5, color='g', linestyle='--')
    axes[1, 2].legend()
    
    plt.tight_layout()
    plt.savefig('output/sa_anomaly_analysis.png', dpi=150, bbox_inches='tight')
    print(f"\nグラフを保存: output/sa_anomaly_analysis.png")
    
    return data


def check_unit_conversion():
    """単位変換の問題を確認"""
    print("\n" + "=" * 70)
    print("単位変換の確認")
    print("=" * 70)
    
    loader = TireDataLoader()
    
    # 生データを直接読み込み
    print("\n生データの確認:")
    df_raw = pd.read_csv('data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv', skiprows=[1])
    
    print(f"  カラム名: {df_raw.columns.tolist()}")
    print(f"\nSAカラムの統計（生データ）:")
    print(f"  最小値: {df_raw['SA'].min():.4f}")
    print(f"  最大値: {df_raw['SA'].max():.4f}")
    print(f"  平均値: {df_raw['SA'].mean():.4f}")
    print(f"  データ型: {df_raw['SA'].dtype}")
    
    # data_loaderを通した後
    data = loader.load_drivebrake_data('data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv')
    
    print(f"\nSAカラムの統計（data_loader経由）:")
    print(f"  最小値: {data['SA'].min():.4f}")
    print(f"  最大値: {data['SA'].max():.4f}")
    print(f"  平均値: {data['SA'].mean():.4f}")
    
    # 変換の確認
    print(f"\n変換の確認:")
    print(f"  生データ最小値: {df_raw['SA'].min():.4f}")
    print(f"  変換後最小値: {data['SA'].min():.4f}")
    print(f"  比率: {data['SA'].min() / df_raw['SA'].min():.4f}")
    
    # 度からラジアンへの変換係数
    deg_to_rad = np.pi / 180
    print(f"\n  度→ラジアン変換係数: {deg_to_rad:.6f}")
    print(f"  実際の変換係数: {data['SA'].min() / df_raw['SA'].min():.6f}")
    
    if abs((data['SA'].min() / df_raw['SA'].min()) - deg_to_rad) < 0.0001:
        print("  ✓ 正しく度→ラジアン変換されています")
    else:
        print("  ✗ 変換に問題がある可能性があります")


def propose_correction():
    """修正方法を提案"""
    print("\n" + "=" * 70)
    print("修正方法の提案")
    print("=" * 70)
    
    loader = TireDataLoader()
    data = loader.load_drivebrake_data('data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv')
    
    print("\n問題:")
    print("  SA範囲: -344 deg ~ 1 deg")
    print("  これは物理的に不可能（タイヤが1回転以上）")
    
    print("\n可能性:")
    print("  1. データ収集時の角度のラップアラウンド")
    print("     -344 deg = -344 + 360 = 16 deg")
    print("     角度が-180～180 degの範囲に正規化されていない")
    
    print("\n  2. センサーの累積誤差")
    print("     長時間のテストで角度が累積")
    
    print("\n  3. 座標系の問題")
    print("     異なる座標系での角度定義")
    
    print("\n推奨される修正方法:")
    print("  オプション1: 角度を-180～180 degに正規化")
    print("    SA_normalized = ((SA + 180) % 360) - 180")
    
    print("\n  オプション2: 純縦力条件でフィルタ")
    print("    |SA| < 5 deg のデータのみ使用")
    
    print("\n  オプション3: 両方を適用")
    print("    1. まず正規化")
    print("    2. 次にフィルタ")
    
    # 正規化の効果を確認
    print("\n正規化の効果:")
    sa_deg = np.rad2deg(data['SA'])
    sa_normalized = ((sa_deg + 180) % 360) - 180
    
    print(f"  元のSA範囲: [{sa_deg.min():.2f}, {sa_deg.max():.2f}] deg")
    print(f"  正規化後: [{sa_normalized.min():.2f}, {sa_normalized.max():.2f}] deg")
    
    # 正規化後の分布
    print(f"\n正規化後の分布:")
    print(f"  |SA| < 5 deg: {len(sa_normalized[np.abs(sa_normalized) < 5])}行 ({len(sa_normalized[np.abs(sa_normalized) < 5])/len(sa_normalized)*100:.1f}%)")
    print(f"  |SA| < 10 deg: {len(sa_normalized[np.abs(sa_normalized) < 10])}行 ({len(sa_normalized[np.abs(sa_normalized) < 10])/len(sa_normalized)*100:.1f}%)")
    print(f"  |SA| < 45 deg: {len(sa_normalized[np.abs(sa_normalized) < 45])}行 ({len(sa_normalized[np.abs(sa_normalized) < 45])/len(sa_normalized)*100:.1f}%)")
    
    # 可視化
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    # 元のSA
    axes[0].hist(sa_deg, bins=100, alpha=0.7, edgecolor='black')
    axes[0].set_xlabel('Slip Angle [deg]')
    axes[0].set_ylabel('Frequency')
    axes[0].set_title('Original SA Distribution')
    axes[0].grid(True, alpha=0.3)
    axes[0].axvline(x=-180, color='r', linestyle='--', label='±180 deg')
    axes[0].axvline(x=180, color='r', linestyle='--')
    axes[0].legend()
    
    # 正規化後のSA
    axes[1].hist(sa_normalized, bins=100, alpha=0.7, edgecolor='black')
    axes[1].set_xlabel('Slip Angle [deg]')
    axes[1].set_ylabel('Frequency')
    axes[1].set_title('Normalized SA Distribution')
    axes[1].grid(True, alpha=0.3)
    axes[1].axvline(x=-5, color='g', linestyle='--', label='±5 deg')
    axes[1].axvline(x=5, color='g', linestyle='--')
    axes[1].legend()
    
    plt.tight_layout()
    plt.savefig('output/sa_normalization_comparison.png', dpi=150, bbox_inches='tight')
    print(f"\nグラフを保存: output/sa_normalization_comparison.png")


def main():
    """メイン実行"""
    print("\n" + "=" * 70)
    print("スリップ角（SA）異常値の詳細分析")
    print("=" * 70)
    print()
    
    # SA分布の分析
    data = analyze_sa_distribution()
    
    # 単位変換の確認
    check_unit_conversion()
    
    # 修正方法の提案
    propose_correction()
    
    print("\n" + "=" * 70)
    print("分析完了")
    print("=" * 70)


if __name__ == '__main__':
    main()
