# `matlab/dl_tire_model/createHyperparameterConfig.m`

## 1. 概要

`createHyperparameterConfig` は、学習・モデル構造・データ分割に関する設定を Name-Value 形式で受け取り、既定値付きの構造体へまとめる関数です。

## 2. ファイルの役割

- 実験用のハイパーパラメータ設定を 1 つの構造体に集約します。
- `inputParser` により型と値域の簡易検証を行います。
- 作成時刻と設定バージョンをメタデータとして付加します。
- 設定内容をコンソールに整形表示します。

## 3. インターフェース

### 3.1 関数シグネチャ

```matlab
config = createHyperparameterConfig(varargin)
```

### 3.2 受け付ける Name-Value 引数

- `learning_rate`（既定値 `0.001`）
- `batch_size`（既定値 `256`）
- `max_epochs`（既定値 `200`）
- `patience`（既定値 `10`）
- `val_frequency`（既定値 `50`）
- `random_seed`（既定値 `42`）
- `model_type`（既定値 `'feedforward'`）
- `hidden_sizes`（既定値 `[128, 64, 32]`）
- `dropout_rate`（既定値 `0.2`）
- `split_ratio`（既定値 `0.8`）

### 3.3 戻り値

- `config`
  - 上記設定に加えて以下を含む構造体
    - `created_at`
    - `config_version`

## 4. 依存関係

- MATLAB 標準関数: `inputParser`, `addParameter`, `parse`, `datetime`, `fprintf`, `num2str`, `isnumeric`, `isvector`, `ischar`, `isstring`, `mod`

## 5. 処理の流れ

1. `inputParser` を生成します。
2. 学習設定・モデル設定・前処理設定の各 Name-Value 引数を検証関数付きで登録します。
3. `parse(p, varargin{:})` で入力を解釈します。
4. `p.Results` から `config` 構造体を組み立てます。
5. `created_at = datetime('now')`, `config_version = '1.0'` を追加します。
6. 設定サマリーを `fprintf` で表示して返します。

## 6. 関数・ローカル関数詳細

### 6.1 `createHyperparameterConfig`

- `batch_size`, `max_epochs`, `patience`, `val_frequency` は正の整数であることを `mod(x,1)==0` で確認しています。
- `dropout_rate` は `0 <= x < 1` を許容します。
- `hidden_sizes` は数値ベクトルであることだけ確認し、各要素の整数性までは検証しません。

## 7. 注意点

- この関数は設定を保存するだけで、`hidden_sizes` や `dropout_rate` を実際のネットワーク層へ反映しません。
- `model_type` の妥当性は文字列判定だけで、`feedforward` / `lstm` 以外でもここでは通ります。
