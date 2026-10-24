# tire_fitting/preprocessor.py

## 役割
タイヤ実験データに対するフィルタ、荷重グループ化、IQR 外れ値除去を行う前処理モジュールです。

## 主なクラス
- `TireDataPreprocessor`

## 主なメソッド
- `filter_data()`: 車速、荷重、スリップ角/率、温度で品質フィルタ
- `group_by_load()`: `Low/Medium/High` の荷重帯へ分類
- `remove_outliers_iqr()`: 四分位範囲法で異常値除去

## 背景・理論
TTC データは純縦・純横条件を厳密には満たさないため、運転条件フィルタが重要です。IQR 法は分布仮定を強く置かずに外れ値を除去でき、タイヤ試験のばらつき処理に向いています。

## 使用ライブラリ
- `pandas`, `numpy`, `logging`

## 入出力
- 入力: TTC DataFrame
- 出力: フィルタ済み/分類済み DataFrame

## パイプライン上の位置
ローダの次段で同定品質を安定化させるデータ整備層です。
