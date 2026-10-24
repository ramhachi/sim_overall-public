# runHyperparameterSearch.m

## 概要
指定した探索空間に対してグリッド探索またはランダム探索を実行し、各試行の学習・評価・ログ保存を自動化する関数です。

## 主要関数・クラス
- `runHyperparameterSearch(X_train, Y_train, X_val, Y_val, norm_params, search_space, search_type, num_trials)`
- `createModelFromConfig`
- `trainTireModel`
- `evaluateModel`
- `logExperiment`

## 理論・背景
探索では多数の組合せを同じ条件で公正に比較する必要があります。本関数は試行生成、学習、評価、記録を標準化し、探索空間全体のサンプリングを機械的に進めます。

## 外部ライブラリ・ツールボックス
- Deep Learning Toolbox

## 入出力
- 入力: 学習/検証データ、正規化係数、探索空間、探索方式、試行数
- 出力: `best_config`、全試行結果の配列

## システム上の役割
探索エンジン本体です。`main_hyperparameter_tuning` から呼ばれ、最良設定探索を自動化します。
