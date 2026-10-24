# `matlab/dl_tire_model/exportModelWithValidation.m`

## 1. 概要

モデルを保存したあとに再読込し、保存前後で予測が一致するか確認する関数です。  
許容誤差を超えた場合は保存したファイルを削除して失敗扱いにします。

## 2. ファイルの役割

- `exportModel` の保存処理に、同一予測性の検証を追加します。
- エクスポート後のモデルが MATLAB 上で元モデルと同じ結果を返すかを確認します。

## 3. インターフェース

### 関数 `exportModelWithValidation(trained_net, norm_params, output_path, X_test)`

- 入力
  - `trained_net`: 学習済みネットワーク
  - `norm_params`: 正規化パラメータ構造体
  - `output_path`: 保存先 `.mat`
  - `X_test`: 任意。生入力のテスト行列。未指定時は `100×5` の乱数を生成
- 出力
  - 戻り値はありません。
  - 検証成功時のみ `output_path` が残ります。

## 4. 依存関係

- 独自関数: `exportModel`
- MATLAB 標準機能: `nargin`, `isempty`, `rng`, `randn`, `predict`, `load`, `isfield`, `max`, `mean`, `delete`

## 5. 処理の流れ

1. 必須引数 3 つが揃っているか確認します。
2. `X_test` が無ければ `rng(42)` 固定で `randn(100, 5)` を作ります。
3. `norm_params` で `X_test` を正規化し、元モデルの予測 `Y_original` を計算します。
4. `exportModel` を呼んで `.mat` を保存します。
5. 保存ファイルを `load` し、`trained_net` と `norm_params` が入っているか確認します。
6. 再読込したモデルで同じ `X_test` に対する予測 `Y_loaded` を計算します。
7. 最大絶対差と平均絶対差を出し、許容値 `1e-10` と比較します。
8. 許容値を超えた場合は保存ファイルを `delete` して `error` を送出します。

## 6. 関数・ローカル関数詳細

### `exportModelWithValidation(trained_net, norm_params, output_path, X_test)`

- ファイル内唯一の関数です。
- 比較は正規化空間のネットワーク出力同士で行います。
- 成功時は最大差・平均差・許容値を表示します。

## 7. 注意点

- 自動生成する `X_test` は `randn` ベースで、学習データ範囲内に制限していません。
- 差分判定は非常に厳しい `1e-10` 固定です。
- 保存ファイルの削除は検証失敗時のみで、`exportModel` 自体の失敗処理は `exportModel` に依存します。

