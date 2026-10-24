# `tire_fitting/evaluator.py`

## 役割

フィット結果の数値評価と可視化を担当する補助モジュールです。  
RMSE / NRMSE / R² / TIC、ピーク摩擦誤差、散布図・残差図の生成をまとめます。

## 実行と入出力

- 直接実行するファイルではありません。
- 主な入力:
  - 実測値配列 `y_true`
  - 予測値配列 `y_pred`
  - スリップ量、荷重グループ、保存先パス
- 主な出力:
  - 評価指標 `dict`
  - PNG 図、または対話表示

## 主要クラス

### `FitEvaluator`

#### `calculate_metrics(y_true, y_pred)`
- RMSE / NRMSE / R² / TIC を一括計算します。
- 戻り値は `{'rmse', 'nrmse', 'r2', 'tic'}` の辞書です。

#### `calculate_peak_friction_error(force_data, fz_data, force_pred)`
- 実測ピーク摩擦係数と同一点の予測摩擦係数を比較し、百分率誤差を返します。

#### `plot_force_vs_slip(...)`
- 実測を散布図、予測を折れ線で描画します。
- `load_groups` を渡すと Low / Medium / High などのグループ別表示になります。
- `save_path` 指定時は親ディレクトリを作成して PNG 保存します。

#### `plot_residuals(y_true, y_pred, save_path=None)`
- 残差 vs 予測値と残差ヒストグラムを 2 面表示します。
- 保存先未指定なら `plt.show()` を使います。

## データフロー

1. 最適化結果から予測配列を受け取ります。
2. 指標計算で適合度を定量化します。
3. 同じ配列から散布図・残差図を生成し、成果物として保存します。

## システム内での位置づけ

`main.py` の評価段階を支える可視化・指標集約層です。  
最適化自体は行わず、結果の妥当性確認とレポート用素材作成に専念します。
