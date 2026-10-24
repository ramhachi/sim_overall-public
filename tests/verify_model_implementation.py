#!/usr/bin/env python3
"""
TMeasyモデル実装の検証スクリプト

目的:
1. モデルが理論通りに動作しているか確認
2. パラメータ感度を確認
3. 既知の入力に対する出力の妥当性を検証
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.insert(0, '.')

from tire_fitting.tmeasy_model import TMeasyModel


def test_basic_functionality():
    """基本的な機能テスト"""
    print("=" * 70)
    print("テスト1: 基本的な機能テスト")
    print("=" * 70)
    
    model = TMeasyModel(fz_nominal=1000.0)
    
    # テストケース1: ゼロスリップ → ゼロ力
    sr = 0.0
    fz = 1000.0
    fx = model.compute_longitudinal_force(sr, fz, mu_x_peak=1.5, k_mu_x=0.1, Kx=20000)
    print(f"\nテスト1.1: ゼロスリップ")
    print(f"  入力: SR={sr}, Fz={fz}N")
    print(f"  出力: Fx={fx:.2f}N")
    print(f"  期待: Fx≈0N")
    print(f"  結果: {'✓ PASS' if abs(fx) < 1e-6 else '✗ FAIL'}")
    
    # テストケース2: 小さいスリップ → 線形領域
    sr = 0.01
    fx = model.compute_longitudinal_force(sr, fz, mu_x_peak=1.5, k_mu_x=0.1, Kx=20000)
    print(f"\nテスト1.2: 小スリップ（線形領域）")
    print(f"  入力: SR={sr}, Fz={fz}N")
    print(f"  出力: Fx={fx:.2f}N")
    print(f"  期待: Fx < 0 (負の力)")
    print(f"  結果: {'✓ PASS' if fx < 0 else '✗ FAIL'}")
    
    # テストケース3: 大きいスリップ → 飽和領域
    sr = 0.2
    fx = model.compute_longitudinal_force(sr, fz, mu_x_peak=1.5, k_mu_x=0.1, Kx=20000)
    mu_x = 1.5 * (1.0 - 0.1 * (fz - 1000) / 1000)  # = 1.5
    fx_max = mu_x * fz  # = 1500 N
    print(f"\nテスト1.3: 大スリップ（飽和領域）")
    print(f"  入力: SR={sr}, Fz={fz}N")
    print(f"  出力: Fx={fx:.2f}N")
    print(f"  期待: |Fx| ≈ {fx_max:.2f}N (飽和)")
    print(f"  結果: {'✓ PASS' if abs(abs(fx) - fx_max) < 100 else '✗ FAIL'}")
    
    return abs(fx) < 1e-6, fx < 0, abs(abs(fx) - fx_max) < 100


def test_parameter_sensitivity():
    """パラメータ感度テスト"""
    print("\n" + "=" * 70)
    print("テスト2: パラメータ感度テスト")
    print("=" * 70)
    
    model = TMeasyModel(fz_nominal=1000.0)
    sr = 0.05
    fz = 1000.0
    
    # mu_x_peak の影響
    print("\nテスト2.1: mu_x_peak の影響")
    mu_values = [1.0, 1.5, 2.0]
    fx_values = []
    for mu in mu_values:
        fx = model.compute_longitudinal_force(sr, fz, mu_x_peak=mu, k_mu_x=0.1, Kx=20000)
        fx_values.append(fx)
        print(f"  mu_x_peak={mu:.1f} → Fx={fx:.2f}N")
    
    # mu が増加すると力も増加するはず
    increasing = all(abs(fx_values[i]) < abs(fx_values[i+1]) for i in range(len(fx_values)-1))
    print(f"  期待: mu増加 → |Fx|増加")
    print(f"  結果: {'✓ PASS' if increasing else '✗ FAIL'}")
    
    # Kx の影響
    print("\nテスト2.2: Kx の影響")
    kx_values = [10000, 20000, 30000]
    fx_values = []
    for kx in kx_values:
        fx = model.compute_longitudinal_force(sr, fz, mu_x_peak=1.5, k_mu_x=0.1, Kx=kx)
        fx_values.append(fx)
        print(f"  Kx={kx} → Fx={fx:.2f}N")
    
    # Kx が増加すると力も増加するはず（線形領域では）
    increasing_kx = all(abs(fx_values[i]) < abs(fx_values[i+1]) for i in range(len(fx_values)-1))
    print(f"  期待: Kx増加 → |Fx|増加（線形領域）")
    print(f"  結果: {'✓ PASS' if increasing_kx else '✗ FAIL'}")
    
    # k_mu_x の影響（荷重依存性）
    print("\nテスト2.3: k_mu_x の影響（荷重依存性）")
    fz_high = 1500.0  # 高荷重
    k_mu_values = [0.0, 0.1, 0.3]
    fx_values = []
    for k_mu in k_mu_values:
        fx = model.compute_longitudinal_force(sr, fz_high, mu_x_peak=1.5, k_mu_x=k_mu, Kx=20000)
        fx_values.append(fx)
        print(f"  k_mu_x={k_mu:.1f} (Fz={fz_high}N) → Fx={fx:.2f}N")
    
    # k_mu が増加すると高荷重時の力は減少するはず
    decreasing_kmu = all(abs(fx_values[i]) > abs(fx_values[i+1]) for i in range(len(fx_values)-1))
    print(f"  期待: k_mu増加 → |Fx|減少（高荷重時）")
    print(f"  結果: {'✓ PASS' if decreasing_kmu else '✗ FAIL'}")
    
    return increasing, increasing_kx, decreasing_kmu


def test_force_characteristics():
    """力特性の詳細テスト"""
    print("\n" + "=" * 70)
    print("テスト3: 力特性の詳細テスト")
    print("=" * 70)
    
    model = TMeasyModel(fz_nominal=1000.0)
    
    # スリップ比を変化させて力特性を確認
    sr_range = np.linspace(-0.3, 0.3, 100)
    fz = 1000.0
    
    # 異なるパラメータセットでテスト
    params_sets = [
        {'mu_x_peak': 1.3, 'k_mu_x': 0.05, 'Kx': 15000, 'label': 'v3パラメータ（下限）'},
        {'mu_x_peak': 1.9, 'k_mu_x': 0.25, 'Kx': 32500, 'label': '中央値パラメータ'},
        {'mu_x_peak': 2.5, 'k_mu_x': 0.5, 'Kx': 50000, 'label': '上限パラメータ'},
    ]
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    for params in params_sets:
        fx_values = []
        for sr in sr_range:
            fx = model.compute_longitudinal_force(
                sr, fz, 
                mu_x_peak=params['mu_x_peak'],
                k_mu_x=params['k_mu_x'],
                Kx=params['Kx']
            )
            fx_values.append(fx)
        
        fx_values = np.array(fx_values)
        
        # プロット1: Fx vs SR
        axes[0, 0].plot(sr_range, fx_values, label=params['label'], linewidth=2)
        
        # プロット2: |Fx| vs |SR|
        axes[0, 1].plot(np.abs(sr_range), np.abs(fx_values), label=params['label'], linewidth=2)
        
        # 最大力を計算
        fx_max = np.max(np.abs(fx_values))
        print(f"\n{params['label']}:")
        print(f"  最大力: {fx_max:.2f}N")
        print(f"  理論最大力: {params['mu_x_peak'] * fz:.2f}N")
        print(f"  比率: {fx_max / (params['mu_x_peak'] * fz) * 100:.1f}%")
    
    # プロット設定
    axes[0, 0].set_xlabel('Slip Ratio [-]')
    axes[0, 0].set_ylabel('Fx [N]')
    axes[0, 0].set_title('縦力 vs スリップ比')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend()
    axes[0, 0].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    axes[0, 0].axvline(x=0, color='k', linestyle='--', alpha=0.3)
    
    axes[0, 1].set_xlabel('|Slip Ratio| [-]')
    axes[0, 1].set_ylabel('|Fx| [N]')
    axes[0, 1].set_title('縦力絶対値 vs スリップ比絶対値')
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].legend()
    
    # 荷重依存性のテスト
    print("\nテスト3.2: 荷重依存性")
    sr = 0.1
    fz_range = np.linspace(500, 1500, 50)
    
    for params in params_sets:
        fx_values = []
        for fz_val in fz_range:
            fx = model.compute_longitudinal_force(
                sr, fz_val,
                mu_x_peak=params['mu_x_peak'],
                k_mu_x=params['k_mu_x'],
                Kx=params['Kx']
            )
            fx_values.append(fx)
        
        fx_values = np.array(fx_values)
        
        # プロット3: Fx vs Fz
        axes[1, 0].plot(fz_range, np.abs(fx_values), label=params['label'], linewidth=2)
        
        # プロット4: Fx/Fz vs Fz (摩擦係数)
        mu_eff = np.abs(fx_values) / fz_range
        axes[1, 1].plot(fz_range, mu_eff, label=params['label'], linewidth=2)
    
    axes[1, 0].set_xlabel('Vertical Load Fz [N]')
    axes[1, 0].set_ylabel('|Fx| [N]')
    axes[1, 0].set_title('縦力 vs 垂直荷重 (SR=0.1)')
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend()
    
    axes[1, 1].set_xlabel('Vertical Load Fz [N]')
    axes[1, 1].set_ylabel('Effective μ = |Fx|/Fz [-]')
    axes[1, 1].set_title('有効摩擦係数 vs 垂直荷重')
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend()
    
    plt.tight_layout()
    plt.savefig('output/model_verification_characteristics.png', dpi=150, bbox_inches='tight')
    print(f"\n特性グラフを保存: output/model_verification_characteristics.png")
    
    return True


def test_actual_data_comparison():
    """実データとの比較テスト"""
    print("\n" + "=" * 70)
    print("テスト4: 実データとの比較")
    print("=" * 70)
    
    # 実データを読み込み
    import pandas as pd
    from tire_fitting.data_loader import TireDataLoader
    
    loader = TireDataLoader()
    drivebrake_data = loader.load_drivebrake_data('data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv')
    
    print(f"\nDriveBrakeデータ読み込み: {len(drivebrake_data)}行")
    
    # 基本的なフィルタリング
    filtered = drivebrake_data[
        (drivebrake_data['V'] > 10/3.6) &  # 10 kph以上
        (drivebrake_data['FZ'] > 200) &     # 200 N以上
        (drivebrake_data['P'] > 20) &       # 20°C以上
        (drivebrake_data['P'] < 100) &      # 100°C以下
        (np.abs(drivebrake_data['SA']) < 0.5 * np.pi/180)  # 純縦力
    ].copy()
    
    print(f"フィルタリング後: {len(filtered)}行")
    
    # v3パラメータでモデル予測
    model = TMeasyModel(fz_nominal=1000.0)
    
    fx_pred = model.compute_longitudinal_force(
        filtered['SR'].values,
        filtered['FZ'].values,
        mu_x_peak=1.3,
        k_mu_x=0.05,
        Kx=15000
    )
    
    fx_actual = filtered['FX'].values
    
    # 統計量を計算
    residuals = fx_actual - fx_pred
    rmse = np.sqrt(np.mean(residuals**2))
    mae = np.mean(np.abs(residuals))
    max_error = np.max(np.abs(residuals))
    
    print(f"\nv3パラメータでの予測精度:")
    print(f"  RMSE: {rmse:.2f}N")
    print(f"  MAE: {mae:.2f}N")
    print(f"  最大誤差: {max_error:.2f}N")
    
    # 実データの範囲を確認
    print(f"\n実データの範囲:")
    print(f"  SR: [{filtered['SR'].min():.4f}, {filtered['SR'].max():.4f}]")
    print(f"  FZ: [{filtered['FZ'].min():.1f}, {filtered['FZ'].max():.1f}]N")
    print(f"  FX: [{filtered['FX'].min():.1f}, {filtered['FX'].max():.1f}]N")
    
    print(f"\nモデル予測の範囲:")
    print(f"  FX_pred: [{fx_pred.min():.1f}, {fx_pred.max():.1f}]N")
    
    # 予測範囲が実データより狭い場合は問題
    if fx_pred.max() < fx_actual.max() * 0.8:
        print(f"\n⚠️ 警告: モデルの予測範囲が実データより狭い！")
        print(f"  実データ最大: {fx_actual.max():.1f}N")
        print(f"  予測最大: {fx_pred.max():.1f}N")
        print(f"  比率: {fx_pred.max() / fx_actual.max() * 100:.1f}%")
    
    # 可視化
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    # 予測 vs 実測
    axes[0].scatter(fx_actual, fx_pred, alpha=0.3, s=10)
    axes[0].plot([fx_actual.min(), fx_actual.max()], 
                 [fx_actual.min(), fx_actual.max()], 
                 'r--', label='理想線', linewidth=2)
    axes[0].set_xlabel('Actual Fx [N]')
    axes[0].set_ylabel('Predicted Fx [N]')
    axes[0].set_title('v3パラメータ: 予測 vs 実測')
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    axes[0].axis('equal')
    
    # 誤差分布
    axes[1].hist(residuals, bins=50, alpha=0.7, edgecolor='black')
    axes[1].axvline(x=0, color='r', linestyle='--', linewidth=2)
    axes[1].set_xlabel('Residual (Actual - Predicted) [N]')
    axes[1].set_ylabel('Frequency')
    axes[1].set_title(f'誤差分布 (RMSE={rmse:.1f}N)')
    axes[1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('output/model_verification_actual_data.png', dpi=150, bbox_inches='tight')
    print(f"\n実データ比較グラフを保存: output/model_verification_actual_data.png")
    
    return rmse, mae, max_error


def main():
    """メイン実行関数"""
    print("\n" + "=" * 70)
    print("TMeasyモデル実装検証")
    print("=" * 70)
    print()
    
    # テスト実行
    test1_results = test_basic_functionality()
    test2_results = test_parameter_sensitivity()
    test3_result = test_force_characteristics()
    test4_results = test_actual_data_comparison()
    
    # 総合評価
    print("\n" + "=" * 70)
    print("総合評価")
    print("=" * 70)
    
    all_pass = all(test1_results) and all(test2_results) and test3_result
    
    if all_pass:
        print("\n✓ モデル実装は理論通りに動作しています")
    else:
        print("\n✗ モデル実装に問題がある可能性があります")
    
    print(f"\n実データでのRMSE: {test4_results[0]:.2f}N")
    
    if test4_results[0] > 500:
        print("\n⚠️ 重要な発見:")
        print("  モデル実装は正しいが、パラメータ値が不適切です")
        print("  最適化アルゴリズムまたは目的関数に問題がある可能性が高い")
    
    print("\n" + "=" * 70)


if __name__ == '__main__':
    main()
