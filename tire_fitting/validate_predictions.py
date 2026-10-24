#!/usr/bin/env python3
"""
予測値と実測値の比較検証スクリプト

学習データからランダムにサンプルを選び、
フィッティングされたモデルの予測値と実測値を比較可視化します。
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import yaml
from pathlib import Path
import argparse

from data_loader import TireDataLoader
from preprocessor import TireDataPreprocessor
from tmeasy_model import TMeasyModel


def load_fitted_parameters(yaml_path):
    """YAMLファイルからフィッティング済みパラメータを読み込む"""
    with open(yaml_path, 'r') as f:
        params = yaml.safe_load(f)
    
    return {
        'mu_x_peak': params['longitudinal']['mu_peak'],
        'k_mu_x': params['longitudinal']['mu_sensitivity'],
        'Kx': params['longitudinal']['Kx'],
        'mu_y_peak': params['lateral']['mu_peak'],
        'k_mu_y': params['lateral']['mu_sensitivity'],
        'Ky': params['lateral']['Ky'],
        'C_gamma': params['camber']['thrust_coefficient'],
        'fz_nominal': params['nominal_load']
    }


def sample_random_points(df, n_samples=50, seed=42):
    """データフレームからランダムにサンプルを抽出"""
    np.random.seed(seed)
    if len(df) <= n_samples:
        return df
    indices = np.random.choice(len(df), size=n_samples, replace=False)
    return df.iloc[indices].copy()


def validate_longitudinal(drivebrake_path, params, output_dir, n_samples=50):
    """縦力の予測値と実測値を比較"""
    print("縦力データの検証中...")
    
    # データ読み込み
    loader = TireDataLoader()
    df = loader.load_drivebrake_data(drivebrake_path)
    
    # 前処理
    preprocessor = TireDataPreprocessor()
    df = preprocessor.filter_data(df, max_sa_for_longitudinal=0.5)
    df = preprocessor.remove_outliers_iqr(df, 'FX')
    
    # ランダムサンプリング
    df_sample = sample_random_points(df, n_samples)
    
    # モデルで予測
    model = TMeasyModel(fz_nominal=params['fz_nominal'])
    fx_pred = model.compute_longitudinal_force(
        df_sample['SR'].values,
        df_sample['FZ'].values,
        params['mu_x_peak'],
        params['k_mu_x'],
        params['Kx']
    )
    
    # 実測値
    fx_actual = df_sample['FX'].values
    
    # 誤差計算
    errors = fx_actual - fx_pred
    rmse = np.sqrt(np.mean(errors**2))
    mae = np.mean(np.abs(errors))
    max_error = np.max(np.abs(errors))
    
    print(f"  サンプル数: {len(df_sample)}")
    print(f"  RMSE: {rmse:.2f} N")
    print(f"  MAE: {mae:.2f} N")
    print(f"  最大誤差: {max_error:.2f} N")
    
    # 可視化
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('縦力の予測精度検証（ランダムサンプル）', fontsize=16, fontweight='bold')
    
    # 1. 予測値 vs 実測値
    ax = axes[0, 0]
    ax.scatter(fx_actual, fx_pred, alpha=0.6, s=50)
    min_val = min(fx_actual.min(), fx_pred.min())
    max_val = max(fx_actual.max(), fx_pred.max())
    ax.plot([min_val, max_val], [min_val, max_val], 'r--', lw=2, label='理想線')
    ax.set_xlabel('実測値 [N]', fontsize=12)
    ax.set_ylabel('予測値 [N]', fontsize=12)
    ax.set_title('予測値 vs 実測値', fontsize=13)
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal', adjustable='box')
    
    # 2. 誤差分布
    ax = axes[0, 1]
    ax.hist(errors, bins=20, alpha=0.7, edgecolor='black')
    ax.axvline(0, color='r', linestyle='--', lw=2, label='ゼロ誤差')
    ax.set_xlabel('誤差 (実測値 - 予測値) [N]', fontsize=12)
    ax.set_ylabel('頻度', fontsize=12)
    ax.set_title(f'誤差分布 (RMSE={rmse:.2f}N)', fontsize=13)
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 3. スリップ比 vs 誤差
    ax = axes[1, 0]
    scatter = ax.scatter(df_sample['SR'].values, errors, 
                        c=df_sample['FZ'].values, cmap='viridis', 
                        alpha=0.6, s=50)
    ax.axhline(0, color='r', linestyle='--', lw=2)
    ax.set_xlabel('スリップ比 [-]', fontsize=12)
    ax.set_ylabel('誤差 [N]', fontsize=12)
    ax.set_title('スリップ比 vs 誤差', fontsize=13)
    ax.grid(True, alpha=0.3)
    cbar = plt.colorbar(scatter, ax=ax)
    cbar.set_label('垂直荷重 [N]', fontsize=10)
    
    # 4. 垂直荷重 vs 誤差
    ax = axes[1, 1]
    ax.scatter(df_sample['FZ'].values, errors, alpha=0.6, s=50)
    ax.axhline(0, color='r', linestyle='--', lw=2)
    ax.set_xlabel('垂直荷重 [N]', fontsize=12)
    ax.set_ylabel('誤差 [N]', fontsize=12)
    ax.set_title('垂直荷重 vs 誤差', fontsize=13)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_path = Path(output_dir) / 'validation_longitudinal.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"  保存: {output_path}")
    plt.close()
    
    return {
        'n_samples': len(df_sample),
        'rmse': rmse,
        'mae': mae,
        'max_error': max_error
    }


def validate_lateral(cornering_path, params, output_dir, n_samples=50):
    """横力の予測値と実測値を比較"""
    print("横力データの検証中...")
    
    # データ読み込み
    loader = TireDataLoader()
    df = loader.load_cornering_data(cornering_path)
    
    # 前処理（横力データは温度フィルタなし、スリップ比条件なし）
    preprocessor = TireDataPreprocessor()
    # 基本的なフィルタのみ適用
    df = df[df['FZ'].abs() >= 500].copy()
    if len(df) > 0:
        df = preprocessor.remove_outliers_iqr(df, 'FY')
    
    # ランダムサンプリング
    df_sample = sample_random_points(df, n_samples)
    
    # モデルで予測
    model = TMeasyModel(fz_nominal=params['fz_nominal'])
    fy_pred = model.compute_lateral_force(
        np.deg2rad(df_sample['SA'].values),
        df_sample['FZ'].values,
        np.deg2rad(df_sample['IA'].values),
        params['mu_y_peak'],
        params['k_mu_y'],
        params['Ky'],
        params['C_gamma']
    )
    
    # 実測値
    fy_actual = df_sample['FY'].values
    
    # 誤差計算
    errors = fy_actual - fy_pred
    rmse = np.sqrt(np.mean(errors**2))
    mae = np.mean(np.abs(errors))
    max_error = np.max(np.abs(errors))
    
    print(f"  サンプル数: {len(df_sample)}")
    print(f"  RMSE: {rmse:.2f} N")
    print(f"  MAE: {mae:.2f} N")
    print(f"  最大誤差: {max_error:.2f} N")
    
    # 可視化
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('横力の予測精度検証（ランダムサンプル）', fontsize=16, fontweight='bold')
    
    # 1. 予測値 vs 実測値
    ax = axes[0, 0]
    ax.scatter(fy_actual, fy_pred, alpha=0.6, s=50)
    min_val = min(fy_actual.min(), fy_pred.min())
    max_val = max(fy_actual.max(), fy_pred.max())
    ax.plot([min_val, max_val], [min_val, max_val], 'r--', lw=2, label='理想線')
    ax.set_xlabel('実測値 [N]', fontsize=12)
    ax.set_ylabel('予測値 [N]', fontsize=12)
    ax.set_title('予測値 vs 実測値', fontsize=13)
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal', adjustable='box')
    
    # 2. 誤差分布
    ax = axes[0, 1]
    ax.hist(errors, bins=20, alpha=0.7, edgecolor='black')
    ax.axvline(0, color='r', linestyle='--', lw=2, label='ゼロ誤差')
    ax.set_xlabel('誤差 (実測値 - 予測値) [N]', fontsize=12)
    ax.set_ylabel('頻度', fontsize=12)
    ax.set_title(f'誤差分布 (RMSE={rmse:.2f}N)', fontsize=13)
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 3. スリップ角 vs 誤差
    ax = axes[1, 0]
    scatter = ax.scatter(df_sample['SA'].values, errors, 
                        c=df_sample['FZ'].values, cmap='viridis', 
                        alpha=0.6, s=50)
    ax.axhline(0, color='r', linestyle='--', lw=2)
    ax.set_xlabel('スリップ角 [deg]', fontsize=12)
    ax.set_ylabel('誤差 [N]', fontsize=12)
    ax.set_title('スリップ角 vs 誤差', fontsize=13)
    ax.grid(True, alpha=0.3)
    cbar = plt.colorbar(scatter, ax=ax)
    cbar.set_label('垂直荷重 [N]', fontsize=10)
    
    # 4. 垂直荷重 vs 誤差
    ax = axes[1, 1]
    ax.scatter(df_sample['FZ'].values, errors, alpha=0.6, s=50)
    ax.axhline(0, color='r', linestyle='--', lw=2)
    ax.set_xlabel('垂直荷重 [N]', fontsize=12)
    ax.set_ylabel('誤差 [N]', fontsize=12)
    ax.set_title('垂直荷重 vs 誤差', fontsize=13)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_path = Path(output_dir) / 'validation_lateral.png'
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"  保存: {output_path}")
    plt.close()
    
    return {
        'n_samples': len(df_sample),
        'rmse': rmse,
        'mae': mae,
        'max_error': max_error
    }


def main():
    parser = argparse.ArgumentParser(
        description='フィッティング結果の予測精度を検証'
    )
    parser.add_argument(
        '--drivebrake',
        type=str,
        default='data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv',
        help='DriveBrake CSVファイルパス'
    )
    parser.add_argument(
        '--cornering',
        type=str,
        default='data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv',
        help='Cornering CSVファイルパス'
    )
    parser.add_argument(
        '--params',
        type=str,
        default='output/tire_params/hoosier_18x6_r20.yaml',
        help='フィッティング済みパラメータYAMLファイル'
    )
    parser.add_argument(
        '--output',
        type=str,
        default='output/tire_params',
        help='出力ディレクトリ'
    )
    parser.add_argument(
        '--n-samples',
        type=int,
        default=50,
        help='ランダムサンプル数'
    )
    
    args = parser.parse_args()
    
    # 出力ディレクトリ作成
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    print("=" * 60)
    print("タイヤモデル予測精度検証")
    print("=" * 60)
    print(f"パラメータファイル: {args.params}")
    print(f"サンプル数: {args.n_samples}")
    print()
    
    # パラメータ読み込み
    params = load_fitted_parameters(args.params)
    print("フィッティング済みパラメータ:")
    print(f"  縦方向: mu_x={params['mu_x_peak']:.4f}, k_mu_x={params['k_mu_x']:.6f}, Kx={params['Kx']:.2f}")
    print(f"  横方向: mu_y={params['mu_y_peak']:.4f}, k_mu_y={params['k_mu_y']:.6f}, Ky={params['Ky']:.2f}")
    print(f"  キャンバー: C_gamma={params['C_gamma']:.4f}")
    print()
    
    # 縦力検証
    print("-" * 60)
    long_stats = validate_longitudinal(
        args.drivebrake, params, output_dir, args.n_samples
    )
    print()
    
    # 横力検証
    print("-" * 60)
    lat_stats = validate_lateral(
        args.cornering, params, output_dir, args.n_samples
    )
    print()
    
    # サマリー
    print("=" * 60)
    print("検証結果サマリー")
    print("=" * 60)
    print(f"縦力:")
    print(f"  RMSE: {long_stats['rmse']:.2f} N")
    print(f"  MAE: {long_stats['mae']:.2f} N")
    print(f"  最大誤差: {long_stats['max_error']:.2f} N")
    print()
    print(f"横力:")
    print(f"  RMSE: {lat_stats['rmse']:.2f} N")
    print(f"  MAE: {lat_stats['mae']:.2f} N")
    print(f"  最大誤差: {lat_stats['max_error']:.2f} N")
    print()
    print(f"出力ディレクトリ: {output_dir}")
    print("  - validation_longitudinal.png")
    print("  - validation_lateral.png")
    print("=" * 60)


if __name__ == '__main__':
    main()
