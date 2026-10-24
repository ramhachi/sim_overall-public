# `matlab/dl_tire_model/exportModelForCpp.m`

## 1. 概要

学習済みモデルを C++ 実装向けのテキスト群へ分解して出力する関数です。  
重み、正規化情報、ネットワーク構造、検証 CSV、C++ テンプレート、README を一括生成します。

## 2. ファイルの役割

- `.mat` モデルを直接読めない C++ 環境向けに、可搬なテキスト表現へ変換します。
- 生成物には「データ」と「使い方説明」の両方が含まれます。

## 3. インターフェース

### 公開関数 `exportModelForCpp(model_path, output_dir)`

- 入力
  - `model_path`: 学習済みモデル `.mat`
  - `output_dir`: 出力先ディレクトリ
- 出力
  - 戻り値はありません。
  - 主な生成物
    - `network_weights.txt`
    - `normalization_params.txt`
    - `network_architecture.txt`
    - `test_data.csv`
    - `dl_tire_model.cpp`
    - `README.txt`

## 4. 依存関係

- 独自関数
  - `exportNetworkWeights`
  - `predictTireForces`
- ファイル内ローカル関数
  - `exportNormalizationParams`
  - `exportNetworkArchitecture`
  - `generateTestData`
  - `generateCppImplementation`
  - `generateReadme`
- MATLAB 標準機能
  - `load`, `isfile`, `exist`, `mkdir`, `fullfile`, `datestr`, `rng`, `unifrnd`

## 5. 処理の流れ

1. `model_path` の存在を確認して `load` します。
2. `output_dir` が無ければ作成します。
3. `exportNetworkWeights` で重みとバイアスを書き出します。
4. ローカル `exportNormalizationParams` で平均値・標準偏差を縦並び形式で保存します。
5. `exportNetworkArchitecture` で各 layer の型と寸法を記録します。
6. `generateTestData` で 50 件の乱数入力と期待出力を CSV 化します。
7. `generateCppImplementation` で C++ 実装テンプレートを生成します。
8. `generateReadme` で連携手順を説明する README を出力します。

## 6. 関数・ローカル関数詳細

### `exportModelForCpp(model_path, output_dir)`

- 全工程の入口です。
- `.mat` からは `model.trained_net` と `model.norm_params` を取り出して各 helper に渡します。

### `exportNormalizationParams(norm_params, output_path)`

- 入出力サイズ、平均値、標準偏差をセクション形式で保存します。
- 各値は 1 行 1 値で `%.15e` 表記です。

### `exportNetworkArchitecture(trained_net, output_path)`

- `trained_net.Layers` を走査し、各 layer のクラス名・名前・寸法を出力します。
- FullyConnected、ReLU、Dropout、BatchNormalization に応じた追加情報を書きます。

### `generateTestData(trained_net, norm_params, output_path)`

- `rng(42)` で乱数を固定し、50 件のサンプルを既定レンジ内から生成します。
- `predictTireForces` の出力を期待値として CSV に保存します。

### `generateCppImplementation(output_dir, output_path)`

- `DenseLayer`、`relu`、`DLTireModel` を含む C++ テンプレートを生成します。
- `load_normalization_params` と `load_network_weights` は `TODO` コメント付きの空実装です。
- 生成されるコードは forward pass と逆正規化の骨格までを含みます。

### `generateReadme(output_dir, output_path)`

- 生成物ごとの説明、C++ 側の読み込み手順、入力特徴量と出力ターゲット一覧を文書化します。
- 検証誤差の目安として `< 0.1% relative difference` を記載しています。

## 7. 注意点

- このファイルが生成する `dl_tire_model.cpp` は完成実装ではなく、ファイル解析関数が未実装です。
- `output_dir` 引数は `generateCppImplementation` と `generateReadme` に渡されますが、両 helper 自身は内容中で `output_dir` を参照していません。
- テストデータ生成には `predictTireForces` が必要です。

