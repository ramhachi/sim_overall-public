# tire_fitting/optimizer.py

## 役割
TMeasy パラメータ最適化と、複数モデル比較フィッタの両方を収めた中核モジュールです。

## 主なクラス
- `TireParameterOptimizer`: TMeasy 専用 2 段階最適化
- `ModelFitter`: TMeasy/Magic Formula の並列フィットと結果キャッシュ

## 主な内容
- `differential_evolution` による大域探索
- `least_squares(method="trf")` による局所洗練
- 縦力・横力で別の境界を持つ
- `ProcessPoolExecutor` を用いた複数モデル並列実行

## 背景・理論
TTC データは非線形性が強いため、大域探索だけでも局所法だけでも不十分です。本ファイルは DE→最小二乗の二段構えでパラメータを安定推定し、その後に RMSE/TIC/R² 比較用の基礎結果を返します。

## 使用ライブラリ
- `numpy`
- `scipy.optimize`
- `concurrent.futures`
- `tire_fitting.models`

## 入出力
- 入力: SR/SA、力、荷重、キャンバー、モデル種別
- 出力: パラメータ辞書、評価指標、複数モデルの結果辞書

## パイプライン上の位置
データ前処理後の「同定エンジン」に相当します。
