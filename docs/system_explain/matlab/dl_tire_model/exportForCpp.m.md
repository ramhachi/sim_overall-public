# `matlab/dl_tire_model/exportForCpp.m`

## 1. 概要

学習済みモデルから、C++ 連携用のルックアップテーブル、正規化情報、ヘッダ、検証データをまとめて生成する関数です。  
このファイル内に、書き出し処理を担うローカル関数も複数含まれています。

## 2. ファイルの役割

- `.mat` モデルを読み込み、C++ 側で使う中間成果物一式を `output_dir` に出力します。
- 推論を都度実行する形ではなく、5 次元グリッド上のルックアップテーブルを生成する流れです。
- デバッグ用に LUT を CSV にも展開します。

## 3. インターフェース

### 公開関数 `exportForCpp(model_path, output_dir, grid_config)`

- 入力
  - `model_path`: 学習済みモデル `.mat` ファイル
  - `output_dir`: 出力先ディレクトリ
  - `grid_config`: 任意。`SR_range`, `SA_range`, `Fz_range`, `IA_range`, `TSTC_range` を持つ構造体
- 出力
  - 戻り値はありません。
  - 主な生成物
    - `tire_lookup_table.mat`
    - `normalization_params.txt`
    - `dl_tire_model.hpp`
    - `test_data.csv`
    - 各軸配列 CSV、各出力グリッド CSV

## 4. 依存関係

- 独自関数
  - `generateLookupTable`
  - `predictTireForces`
- ファイル内ローカル関数
  - `exportNormalizationParams`
  - `generateCppHeader`
  - `generateTestData`
  - `exportLookupTableToCSV`
- MATLAB 標準機能
  - `load`, `isfile`, `mkdir`, `fullfile`, `datestr`, `writematrix`, `rng`, `unifrnd`

## 5. 処理の流れ

1. `model_path` の存在を確認し、`load` で `trained_net` と `norm_params` を含む構造体を読み込みます。
2. 出力ディレクトリが無ければ作成します。
3. `grid_config` が未指定なら、固定既定値
   - `SR [-0.2, 0.2, 40]`
   - `SA [-6, 6, 50]`
   - `Fz [200, 1300, 30]`
   - `IA [0, 4, 15]`
   - `TSTC [55, 80, 15]`
   を設定します。
4. `generateLookupTable` を呼び、`tire_lookup_table.mat` を生成します。
5. 正規化パラメータ、C++ ヘッダ、テストデータを順に出力します。
6. 最後に LUT を CSV に展開して、デバッグや外部確認に使える形にします。

## 6. 関数・ローカル関数詳細

### `exportForCpp(model_path, output_dir, grid_config)`

- 全体のオーケストレーションを行う公開関数です。
- モデル読込後は `model.trained_net` と `model.norm_params` を各 helper に渡します。

### `exportNormalizationParams(norm_params, output_path)`

- 入出力の平均値・標準偏差を 1 行 CSV 風フォーマットで書き出します。
- 入力は `SR, SA, Fz, IA, TSTC`、出力は `Fx, Fy, Mx, Mz` の順です。

### `generateCppHeader(norm_params, grid_config, output_path)`

- `fsae::physics::tire` 名前空間の C++ ヘッダを生成します。
- 正規化定数、`DLTireOutput`、`DLTireModel` 宣言、5 次元補間 helper の宣言を書き出します。

### `generateTestData(trained_net, norm_params, output_path)`

- 固定 seed `rng(42)` で 20 点のランダム入力を作り、`predictTireForces` で期待出力を生成します。
- `SR,SA,Fz,IA,TSTC,Fx,Fy,Mx,Mz` の CSV として保存します。

### `exportLookupTableToCSV(lut_path, output_dir)`

- `lookup_table` 構造体をロードし、各軸配列と各出力グリッドを個別 CSV に書き出します。
- 出力グリッドは `(:)` でフラット化して保存します。

## 7. 注意点

- `.mat` ファイル内に `trained_net` と `norm_params` が存在する前提です。
- `generateLookupTable` と `predictTireForces` はこのファイル内に実装されていないため、別ファイルが必要です。
- 生成されるのは C++ 実装本体ではなく、ヘッダと LUT データ中心の連携成果物です。

