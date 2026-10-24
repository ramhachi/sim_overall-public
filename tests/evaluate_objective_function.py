#!/usr/bin/env python3
"""
目的関数の値を直接評価するスクリプト

目的:
異なるパラメータ値で目的関数を評価し、
下限パラメータが本当に最適解なのかを確認する
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import sys
sys.path.insert(0, '.')

from tire_fitting.tmeasy_model import TMeasyModel
from tire_fitting.data_loader import TireDataLoader
from tire_fitting.preprocessor import TireDataPreprocessor


def load_and_filter_data():
    """データを読み込んでフィルタリング"""
    print("=" * 70)
    print("データ読み込みとフィルタリング")
    print("=" * 70)
    
    # データ読み込み
    loader = TireDataLoader()
    drivebrake_data = loader.load_drivebrake_data(
        'data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv'
    )
    
    print(f"\nDriveBrakeデータ: {len(drivebrake_data)}行")
    
    # フィルタリング（v3/v4と同じ条件）
    preprocessor = TireDataPreprocessor()
    filtered = preprocessor.filter_data(
        drivebrake_data,
        min_speed=10.0,  # kph
        min_fz=200.0,
        temp_range=(20.0, 100.0),
        max_sa_for_longitudinal=0.5  # deg
    )
    
    print(f"フィルタリング後: {len(filtered)}行 ({len(filtered)/len(drivebrake_data)*100:.1f}%)")
    
    # 外れ値除去
    filtered = preprocessor.remove_outliers_iqr(filtered, column='FX')
    
    print(f"外れ値除去後: {len(filtered)}行")
    
    return filtered


def evaluate_objective(sr_data, fx_data, fz_data, params, model):
    """目的関数を評価"""
    mu_x_peak, k_mu_x, Kx = params
    
    # 予測
    fx_pred = model.compute_longitudinal_force(
        sr_data, fz_data, mu_x_peak, k_mu_x, Kx
    )
    
    # 絶対残差（v4の目的関数）
    residuals = fx_data - fx_pred
    objective_value = np.sum(residuals**2)
    
    # RMSE
    rmse = np.sqrt(np.mean(residuals**2))
    
    # 最大誤差
    max_error = np.max(np.abs(residuals))
    
    # 予測範囲
    fx_pred_range = (fx_pred.min(), fx_pred.max())
    
    return {
        'objective': objective_value,
        'rmse': rmse,
        'max_error': max_error,
        'fx_pred_range': fx_pred_range,
        'fx_pred': fx_pred
    }


def main():
    """メイン実行"""
    print("\n" + "=" * 70)
    print("目的関数の直接評価")
    print("=" * 70)
    print()
    
    # データ読み込み
    data = load_and_filter_data()
    
    sr_data = data['SR'].values
    fx_data = data['FX'].values
    fz_data = data['FZ'].values
    
    print(f"\n実データの範囲:")
    print(f"  SR: [{sr_data.min():.4f}, {sr_data.max():.4f}]")
    print(f"  FZ: [{fz_data.min():.1f}, {fz_data.max():.1f}]N")
    print(f"  FX: [{fx_data.min():.1f}, {fx_data.max():.1f}]N")
    
    # モデル初期化
    model = TMeasyModel(fz_nominal=1000.0)
    
    # テストするパラメータセット
    param_sets = [
        {
            'name': '下限（v3/v4結果）',
            'params': [1.3, 0.05, 15000],
            'color': 'red'
        },
        {
            'name': '中央値',
            'params': [1.9, 0.275, 32500],
            'color': 'green'
        },
        {
            'name': '上限',
            'params': [2.5, 0.5, 50000],
            'color': 'blue'
        },
        {
            'name': '推定値（データ駆動）',
            'params': [2.0, 0.15, 28000],
            'color': 'orange'
        },
    ]
    
    print("\n" + "=" * 70)
    print("パラメータセットの評価")
    print("=" * 70)
    
    results = []
    
    for param_set in param_sets:
        print(f"\n{param_set['name']}:")
        print(f"  mu_x_peak = {param_set['params'][0]:.2f}")
        print(f"  k_mu_x = {param_set['params'][1]:.3f}")
        print(f"  Kx = {param_set['params'][2]:.0f}N")
        
        result = evaluate_objective(
            sr_data, fx_data, fz_data,
            param_set['params'], model
        )
        
        print(f"  目的関数値: {result['objective']:.2e}")
        print(f"  RMSE: {result['rmse']:.2f}N")
        print(f"  最大誤差: {result['max_error']:.2f}N")
        print(f"  予測範囲: [{result['fx_pred_range'][0]:.1f}, {result['fx_pred_range'][1]:.1f}]N")
        
        results.append({
            'name': param_set['name'],
            'params': param_set['params'],
            'color': param_set['color'],
            **result
        })
    
    # 最良のパラメータを特定
    print("\n" + "=" * 70)
    print("結果サマリー")
    print("=" * 70)
    
    best_idx = np.argmin([r['objective'] for r in results])
    print(f"\n最良のパラメータセット: {results[best_idx]['name']}")
    print(f"  目的関数値: {results[best_idx]['objective']:.2e}")
    print(f"  RMSE: {results[best_idx]['rmse']:.2f}N")
    
    # 可視化
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # プロット1: 目的関数値の比較
    names = [r['name'] for r in results]
    objectives = [r['objective'] for r in results]
    colors = [r['color'] for r in results]
    
    axes[0, 0].bar(range(len(names)), objectives, color=colors, alpha=0.7)
    axes[0, 0].set_xticks(range(len(names)))
    axes[0, 0].set_xticklabels(names, rotation=45, ha='right')
    axes[0, 0].set_ylabel('Objective Function Value')
    axes[0, 0].set_title('Objective Function Comparison')
    axes[0, 0].set_yscale('log')
    axes[0, 0].grid(True, alpha=0.3)
    
    # プロット2: RMSE比較
    rmses = [r['rmse'] for r in results]
    
    axes[0, 1].bar(range(len(names)), rmses, color=colors, alpha=0.7)
    axes[0, 1].set_xticks(range(len(names)))
    axes[0, 1].set_xticklabels(names, rotation=45, ha='right')
    axes[0, 1].set_ylabel('RMSE [N]')
    axes[0, 1].set_title('RMSE Comparison')
    axes[0, 1].axhline(y=100, color='r', linestyle='--', label='Target (100N)')
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].legend()
    
    # プロット3: 予測 vs 実測（最良のパラメータ）
    best_result = results[best_idx]
    fx_pred_best = best_result['fx_pred']
    
    axes[1, 0].scatter(fx_data, fx_pred_best, alpha=0.3, s=10, color=best_result['color'])
    axes[1, 0].plot([fx_data.min(), fx_data.max()],
                    [fx_data.min(), fx_data.max()],
                    'r--', label='Ideal', linewidth=2)
    axes[1, 0].set_xlabel('Actual Fx [N]')
    axes[1, 0].set_ylabel('Predicted Fx [N]')
    axes[1, 0].set_title(f'Best: {best_result["name"]}')
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend()
    axes[1, 0].axis('equal')
    
    # プロット4: 予測 vs 実測（下限パラメータ）
    lower_result = results[0]  # 下限は最初
    fx_pred_lower = lower_result['fx_pred']
    
    axes[1, 1].scatter(fx_data, fx_pred_lower, alpha=0.3, s=10, color=lower_result['color'])
    axes[1, 1].plot([fx_data.min(), fx_data.max()],
                    [fx_data.min(), fx_data.max()],
                    'r--', label='Ideal', linewidth=2)
    axes[1, 1].set_xlabel('Actual Fx [N]')
    axes[1, 1].set_ylabel('Predicted Fx [N]')
    axes[1, 1].set_title(f'Current (v3/v4): {lower_result["name"]}')
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend()
    axes[1, 1].axis('equal')
    
    plt.tight_layout()
    plt.savefig('output/objective_function_evaluation.png', dpi=150, bbox_inches='tight')
    print(f"\nグラフを保存: output/objective_function_evaluation.png")
    
    # 結論
    print("\n" + "=" * 70)
    print("結論")
    print("=" * 70)
    
    if best_idx == 0:
        print("\n⚠️ 下限パラメータが最良の目的関数値を持つ")
        print("   → データの問題またはモデルの限界")
        print("   → フィルタリング条件を見直すべき")
    else:
        print(f"\n✓ {results[best_idx]['name']}が最良")
        print("   → 最適化アルゴリズムの問題")
        print("   → DEが局所最適解に陥っている")
        print(f"   → 改善の余地: RMSE {results[0]['rmse']:.1f}N → {results[best_idx]['rmse']:.1f}N")
    
    print("\n" + "=" * 70)


if __name__ == '__main__':
    main()
