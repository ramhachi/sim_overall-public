# `matlab/dl_tire_model/evaluateModel.m`

## 1. 概要

学習済みネットワークを検証データで評価し、`Fx/Fy/Mx/Mz` ごとの指標と閾値判定を返す関数です。  
予測、逆正規化、指標計算、要求判定、サマリ表示までを一括で行います。

## 2. ファイルの役割

- 検証入力に対する予測値を生成します。
- 正規化空間の出力を物理単位へ戻して、`RMSE`、`R²`、`TIC`、`MAE` を計算します。
- `Fx/Fy` の `RMSE < 100`、全出力の `R² > 0.95` を満たすかを `metrics` に格納します。

## 3. インターフェース

### 関数 `metrics = evaluateModel(trained_net, X_val, Y_val, norm_params)`

- 入力
  - `trained_net`: 学習済みニューラルネットワーク
  - `X_val`: 正規化済み検証入力 `N×5`
  - `Y_val`: 正規化済み検証教師 `N×4`
  - `norm_params`: `output_mean`, `output_std` を含む正規化構造体
- 出力
  - `metrics`: 以下を含む構造体
    - `Fx`, `Fy`, `Mx`, `Mz`
      - `RMSE`, `R2`, `TIC`, `MAE`
    - `meets_requirements`
    - `threshold_flags`

## 4. 依存関係

- 独自関数: `denormalizeData`, `computeRMSE`, `computeR2`, `computeTIC`
- MATLAB 標準機能: `predict`, `mean`, `abs`, `struct2cell`, `fprintf`

## 5. 処理の流れ

1. `predict(trained_net, X_val)` で正規化空間の予測値 `Y_pred_norm` を得ます。
2. `denormalizeData` を使って予測値と教師値を物理単位へ戻します。
3. `computeRMSE/R2/TIC` と `mean(abs(...))` で各出力の指標を計算します。
4. 出力名 `Fx/Fy/Mx/Mz` ごとに `metrics.(name)` へ格納します。
5. `Fx/Fy` の RMSE と全出力の R² を閾値と比較し、`threshold_flags` を作ります。
6. すべての flag が false なら `metrics.meets_requirements = true` にします。
7. 最後に表形式のサマリを標準出力へ表示します。

## 6. 関数・ローカル関数詳細

### `evaluateModel(trained_net, X_val, Y_val, norm_params)`

- ファイル内唯一の関数です。
- 指標計算は出力 4 列をまとめてベクトル化して行い、その後構造体へ分配します。
- `MAE` のみこの関数内で `mean(abs(Y_actual - Y_pred), 1)` として直接計算しています。
- 閾値未達時は、どの項目が閾値を外れたかを個別メッセージとして表示します。

## 7. 注意点

- 入力 `X_val` と `Y_val` はコメント上で「正規化済み」であることが前提です。
- 閾値判定は `Fx/Fy` の RMSE と各出力の R² のみで、`Mx/Mz` の RMSE しきい値判定は実装されていません。
- `norm_params` では出力側の平均・標準偏差しか参照しません。

