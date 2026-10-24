# `matlab/dl_tire_model/exportNetworkWeights.m`

## 1. 概要

学習済みニューラルネットワークから、全結合層の重みとバイアスをテキスト形式で書き出す関数です。  
C++ など MATLAB 外部で同じネットワークを再実装する用途を想定しています。

## 2. ファイルの役割

- `trained_net.Layers` を走査し、Fully Connected layer だけを抽出します。
- 層ごとに寸法情報、重み行列、バイアスベクトルを人間可読な形式で保存します。

## 3. インターフェース

### 関数 `exportNetworkWeights(trained_net, output_path)`

- 入力
  - `trained_net`: 学習済みネットワーク
  - `output_path`: 出力先テキストファイル
- 出力
  - 戻り値はありません。
  - `LAYER_n_WEIGHTS` / `LAYER_n_BIASES` 形式のテキストを生成します。

## 4. 依存関係

- MATLAB 標準機能: `fopen`, `fprintf`, `datestr`, `isa`, `size`, `dir`, `fclose`
- 判定対象クラス: `nnet.cnn.layer.FullyConnectedLayer`

## 5. 処理の流れ

1. 出力ファイルを開き、失敗時は `error` を出します。
2. ヘッダとして生成日時、保存形式、入出力の意味を記述します。
3. `trained_net.Layers` を走査し、全結合層数を数えます。
4. もう一度 layer を走査し、Fully Connected layer ごとに
   - 層番号
   - 入力数・出力数
   - 重み行列
   - バイアス
   を書き出します。
5. ファイルを閉じ、サイズと層数を表示します。

## 6. 関数・ローカル関数詳細

### `exportNetworkWeights(trained_net, output_path)`

- ファイル内唯一の関数です。
- 重みは `layer.Weights` を `Output × Input` 行列としてそのまま出力します。
- 出力順は行優先で、各行が出力ニューロン 1 個分の重み列です。

## 7. 注意点

- 全結合層以外のパラメータは出力しません。
- ファイル内にはネットワーク全体の forward 規則は含まれず、別途活性化関数の扱いが必要です。
- 既存ファイルがある場合は開いた時点で上書きされます。

