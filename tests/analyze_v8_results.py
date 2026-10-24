#!/usr/bin/env python3
"""
v8結果の分析

純縦力条件フィルタ（|SA| < 5 deg）を適用したv8の結果を分析する。
なぜ縦力パラメータが下限に張り付いたままなのかを調査する。
"""

import sys
sys.path.insert(0, '.')

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from tire_fitting import TireDataLoader, TMeasyModel

# データ読み込み
loader = TireDataLoader()
drivebrake_df = loader.load_drivebrake_data('data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv')

# v8のフィルタリングを適用
required_cols = ['SR', 'FX', 'FZ', 'SA']
drivebrake_filtered = drivebrake_df.dropna(subset=required_cols).copy()
drivebrake_filtered = drivebrake_filtered[drivebrake_filtered['FZ'] > 0].copy()

# 純縦力条件フィルタ
sa_threshold_rad = np.deg2rad(5.0)
drivebrake_filtered = drivebrake_filtered[
    np.abs(drivebrake_filtered['SA']) < sa_threshold_rad
].copy()

print("=" * 70)
print("v8 データ分析")
print("=" * 70)
print()
print(f"データ量: {len(drivebrake_filtered)} 行")
print()

# SR範囲の分析
sr_data = drivebrake_filtered['SR'].values
fx_data = drivebrake_filtered['FX'].values
fz_data = drivebrake_filtered['FZ'].values

print("SR範囲:")
print(f"  最小値: {sr_data.min():.4f}")
print(f"  最大値: {sr_data.max():.4f}")
print(f"  平均値: {sr_data.mean():.4f}")
print(f"  標準偏差: {sr_data.std():.4f}")
print()

# SR分布の分析
sr_bins = [-1.0, -0.2, -0.1, -0.05, 0.0, 0.05, 0.1, 0.2, 1.0]
sr_hist, _ = np.histogram(sr_data, bins=sr_bins)
print("SR分布:")
for i in range(len(sr_bins) - 1):
    print(f"  [{sr_bins[i]:.2f}, {sr_bins[i+1]:.2f}): {sr_hist[i]} 行 ({100 * sr_hist[i] / len(sr_data):.1f}%)")
print()

# FZ範囲の分析
print("FZ範囲:")
print(f"  最小値: {fz_data.min():.1f} N")
print(f"  最大値: {fz_data.max():.1f} N")
print(f"  平均値: {fz_data.mean():.1f} N")
print(f"  標準偏差: {fz_data.std():.1f} N")
print()

# FX範囲の分析
print("FX範囲:")
print(f"  最小値: {fx_data.min():.1f} N")
print(f"  最大値: {fx_data.max():.1f} N")
print(f"  平均値: {fx_data.mean():.1f} N")
print(f"  標準偏差: {fx_data.std():.1f} N")
print()

# 摩擦係数の分析
mu_data = np.abs(fx_data / fz_data)
print("摩擦係数 (|FX| / FZ):")
print(f"  最小値: {mu_data.min():.4f}")
print(f"  最大値: {mu_data.max():.4f}")
print(f"  平均値: {mu_data.mean():.4f}")
print(f"  標準偏差: {mu_data.std():.4f}")
print(f"  95パーセンタイル: {np.percentile(mu_data, 95):.4f}")
print(f"  99パーセンタイル: {np.percentile(mu_data, 99):.4f}")
print()

# ピーク摩擦係数の推定
# 各FZ範囲でのピーク摩擦係数を計算
fz_ranges = [
    (600, 900, "Low"),
    (900, 1100, "Medium"),
    (1100, 1400, "High")
]

print("各荷重範囲でのピーク摩擦係数:")
for fz_min, fz_max, label in fz_ranges:
    mask = (fz_data >= fz_min) & (fz_data <= fz_max)
    if mask.sum() > 0:
        mu_range = mu_data[mask]
        peak_mu = np.percentile(mu_range, 95)
        print(f"  {label} ({fz_min}-{fz_max} N): {peak_mu:.4f} (95%ile)")
print()

# TMeasyモデルでの予測
model = TMeasyModel(fz_nominal=1000.0)

# v8のパラメータ
mu_x_peak = 1.30
k_mu_x = 0.05
Kx = 15000.0

fx_pred = model.compute_longitudinal_force(sr_data, fz_data, mu_x_peak, k_mu_x, Kx)
residuals = fx_data - fx_pred
rmse = np.sqrt(np.mean(residuals**2))

print("v8パラメータでの予測:")
print(f"  mu_x_peak = {mu_x_peak:.4f}")
print(f"  k_mu_x = {k_mu_x:.4f}")
print(f"  Kx = {Kx:.2f} N")
print(f"  RMSE = {rmse:.2f} N")
print()

# より高いmu_x_peakでの予測
print("異なるmu_x_peakでの予測:")
for test_mu in [1.5, 1.8, 2.0, 2.2, 2.5]:
    fx_pred_test = model.compute_longitudinal_force(sr_data, fz_data, test_mu, k_mu_x, Kx)
    residuals_test = fx_data - fx_pred_test
    rmse_test = np.sqrt(np.mean(residuals_test**2))
    print(f"  mu_x_peak = {test_mu:.2f}: RMSE = {rmse_test:.2f} N")
print()

# グラフ作成
fig, axes = plt.subplots(2, 2, figsize=(14, 10))

# 1. SR vs FX (実データ)
ax = axes[0, 0]
scatter = ax.scatter(sr_data, fx_data, c=fz_data, cmap='viridis', alpha=0.3, s=1)
ax.set_xlabel('Slip Ratio [-]')
ax.set_ylabel('Longitudinal Force [N]')
ax.set_title('v8: SR vs FX (実データ)')
ax.grid(True, alpha=0.3)
plt.colorbar(scatter, ax=ax, label='FZ [N]')

# 2. SR vs 摩擦係数
ax = axes[0, 1]
scatter = ax.scatter(sr_data, mu_data, c=fz_data, cmap='viridis', alpha=0.3, s=1)
ax.set_xlabel('Slip Ratio [-]')
ax.set_ylabel('Friction Coefficient |FX|/FZ [-]')
ax.set_title('v8: SR vs 摩擦係数')
ax.axhline(y=1.3, color='r', linestyle='--', label='mu_x_peak = 1.3 (v8)')
ax.axhline(y=2.0, color='g', linestyle='--', label='mu_x_peak = 2.0 (期待値)')
ax.grid(True, alpha=0.3)
ax.legend()
plt.colorbar(scatter, ax=ax, label='FZ [N]')

# 3. 摩擦係数のヒストグラム
ax = axes[1, 0]
ax.hist(mu_data, bins=50, alpha=0.7, edgecolor='black')
ax.axvline(x=1.3, color='r', linestyle='--', linewidth=2, label='mu_x_peak = 1.3 (v8)')
ax.axvline(x=np.percentile(mu_data, 95), color='g', linestyle='--', linewidth=2, 
           label=f'95%ile = {np.percentile(mu_data, 95):.2f}')
ax.set_xlabel('Friction Coefficient |FX|/FZ [-]')
ax.set_ylabel('Frequency')
ax.set_title('v8: 摩擦係数の分布')
ax.grid(True, alpha=0.3)
ax.legend()

# 4. 残差分析
ax = axes[1, 1]
ax.scatter(fx_pred, residuals, alpha=0.3, s=1)
ax.axhline(y=0, color='r', linestyle='--')
ax.set_xlabel('Predicted FX [N]')
ax.set_ylabel('Residual (Actual - Predicted) [N]')
ax.set_title(f'v8: 残差分析 (RMSE = {rmse:.2f} N)')
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('output/v8_data_analysis.png', dpi=150, bbox_inches='tight')
print(f"グラフを保存しました: output/v8_data_analysis.png")
print()

print("=" * 70)
print("分析完了")
print("=" * 70)
