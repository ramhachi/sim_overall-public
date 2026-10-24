# tire_fitting/evaluator.py

## 役割
フィット品質の数値評価と可視化を行います。

## 主なクラス
- `FitEvaluator`

## 主なメソッド
- `calculate_metrics()`: RMSE/NRMSE/R²/TIC
- `calculate_peak_friction_error()`
- `plot_force_vs_slip()`
- `plot_residuals()`

## 背景・理論
RMSE は絶対誤差、NRMSE はスケール正規化、R² は説明率、TIC は予測と実測の相対的乖離を示します。タイヤ同定の採否判断に必要な基礎指標を一箇所に集約しています。

## 使用ライブラリ
- `numpy`, `matplotlib`, `pathlib`

## 入出力
- 入力: 実測配列と予測配列
- 出力: 指標辞書、散布図/残差図

## パイプライン上の位置
最適化結果の採点器です。
