# `matlab/dl_tire_model/export_quick_reference.m`

## 1. 概要

モデル保存・再読込・LUT 生成・学習からの一連手順を例示するクイックリファレンス用スクリプトです。  
実装本体ではなく、既存関数の呼び方をまとめたサンプル集になっています。

## 2. ファイルの役割

- エクスポート関連関数の典型的な使用例をまとめます。
- 単発の保存例から、学習→評価→保存→LUT 生成までの流れを 6 セクションで提示します。

## 3. インターフェース

### スクリプトとしての入出力

- 引数・戻り値はありません。
- セクション 1〜5 では、`trained_net` と `norm_params` が既にワークスペースに存在する前提です。
- 主な出力は、各セクションで呼ばれる関数が生成する `.mat`、CSV、LUT などです。

## 4. 依存関係

- 独自関数
  - `exportModel`
  - `exportModelWithValidation`
  - `predictTireForces`
  - `loadAndPredict`
  - `generateLookupTable`
  - `preprocessData`
  - `createTireModel`
  - `trainTireModel`
  - `evaluateModel`
- MATLAB 標準機能
  - `load`, `interpn`, `fprintf`

## 5. 処理の流れ

### セクション 1: Basic model export

- `output/tire_model.mat` へ `exportModel` を実行する最小例です。

### セクション 2: Export with validation

- `exportModelWithValidation` の自動検証付き保存例です。
- さらに 1 行の `X_test` を明示指定する呼び方も示します。

### セクション 3: Load and make predictions

- `load` で `.mat` を読み、`predictTireForces` を直接使う例を示します。
- `loadAndPredict` を使う簡易呼び出しと、3 行のバッチ推論例も含みます。

### セクション 4: Generate lookup table for C++ integration

- `generateLookupTable` を既定グリッドで呼ぶ例と、`grid_config` を明示構築する例を示します。

### セクション 5: Use lookup table for interpolation

- 保存済み LUT をロードし、`interpn` で `Fx/Fy/Mx/Mz` を個別補間する例を示します。
- 補間点は `SR=0.05`, `SA=5`, `Fz=1500`, `IA=-2`, `TSTC=30` です。

### セクション 6: Complete workflow from training to export

- データ読込、前処理、モデル作成、学習、評価、保存、LUT 生成までを順に並べています。
- `data_paths` には DriveBrake と Cornering の 2 CSV を指定しています。

## 6. 関数・ローカル関数詳細

このファイルはスクリプトで、ローカル関数はありません。  
内容はすべて使用例であり、各セクションが個別のワークフローサンプルです。

## 7. 注意点

- セクション 1〜5 は必要変数や関数が既に利用可能な前提で、そのまま単独実行すると未定義変数エラーになる可能性があります。
- 例示用スクリプトなので、例の前後で変数名 `output_path` を上書きしています。
- 補間例では `interpn(..., 'linear')` を各出力に対して個別に呼んでいます。

