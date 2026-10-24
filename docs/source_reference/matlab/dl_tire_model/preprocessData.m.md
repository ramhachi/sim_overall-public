# preprocessData.m

## 概要
データ読込から特徴量抽出、欠損除去、正規化、学習/検証分割までを一括で行う前処理パイプライン関数です。学習用データセット準備の標準ルートになっています。

## 主要関数・クラス
- `preprocessData(data_paths, config)`
- `loadTireData`
- `extractFeatures`
- `extractTargets`
- `removeMissingData`
- `normalizeData`
- `splitData`

## 理論・背景
前処理の一貫性はモデル比較の公平性に直結します。本関数では特徴抽出順、欠損除去条件、正規化方法、分割比率を固定化し、複数の学習実験が同じ前提条件で行われるようにします。入出力の平均・標準偏差も同時に保存されるため、推論・評価・外部展開まで一貫したスケールで運用できます。

## 外部ライブラリ・ツールボックス
- MATLAB 標準テーブル処理
- MATLAB 標準数値演算

## 入出力
- 入力:
  - `data_paths`: CSV パス配列
  - `config`: `split_ratio` など
- 出力:
  - `X_train`, `Y_train`
  - `X_val`, `Y_val`
  - `norm_params`

## システム上の役割
学習パイプラインの基礎層です。すべての上位ワークフローが安定した前処理を共有するための共通関数として機能します。
