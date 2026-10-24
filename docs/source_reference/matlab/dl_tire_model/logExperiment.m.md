# logExperiment.m

## 概要
探索試行ごとの設定、性能指標、学習情報をログとして保存する関数です。戻り値として実験 ID を返します。

## 主要関数・クラス
- `logExperiment(config, metrics, training_info, log_dir)`

## 理論・背景
ハイパーパラメータ探索では、どの設定がどの結果を生んだかを確実に残す必要があります。本関数は再現性と監査性を担保するための記録機構です。

## 外部ライブラリ・ツールボックス
- MATLAB 標準 I/O

## 入出力
- 入力: 設定構造体、評価指標、学習情報、ログ保存先
- 出力: `experiment_id`

## システム上の役割
探索管理の中核です。`runHyperparameterSearch` と `compareExperiments` の橋渡しを担います。
