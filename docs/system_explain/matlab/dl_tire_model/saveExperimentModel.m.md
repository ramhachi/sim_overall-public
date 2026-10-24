# `matlab/dl_tire_model/saveExperimentModel.m`

## 1. 概要

`saveExperimentModel` は、学習済みネットワークと正規化パラメータを実験 ID 付きで保存する関数です。加えて、同じ実験 ID に対応する予測用関数ハンドルも別 `.mat` ファイルとして保存します。

## 2. ファイルの役割

この関数の役割は、学習結果を `experiment_id` ベースの一意な名前で永続化し、既存モデルの上書きを避けることです。モデル本体の保存と、`predictTireForces` を呼ぶための `predict_fn` 保存を同時に行います。

## 3. インターフェース

### 関数シグネチャ

```matlab
model_path = saveExperimentModel(trained_net, norm_params, experiment_id, model_dir)
```

### 引数

- `trained_net`
  - 学習済みニューラルネットワーク。
- `norm_params`
  - 正規化パラメータの構造体。
- `experiment_id`
  - 保存ファイル名の先頭に使う実験識別子。
- `model_dir`
  - 保存先ディレクトリ。省略時は `'experiments/models'`。

### 戻り値

- `model_path`
  - 保存したモデル本体ファイルのフルパス。

### 保存される内容

- `<experiment_id>_model.mat`
  - `trained_net`, `norm_params`, `experiment_id`
- `<experiment_id>_predict.mat`
  - `predict_fn`, `experiment_id`

## 4. 依存関係

- MATLAB 組み込み
  - `nargin`, `exist`, `mkdir`, `sprintf`, `fullfile`, `save`, `fprintf`
- プロジェクト内関数
  - `predictTireForces`
    - `predict_fn = @(X_raw) predictTireForces(trained_net, X_raw, norm_params);` の形で関数ハンドル化されます。

## 5. 処理の流れ

1. `model_dir` が未指定なら `'experiments/models'` を既定値に設定します。
2. 保存先ディレクトリが存在しない場合は `mkdir(model_dir)` で作成します。
3. `sprintf('%s_model.mat', experiment_id)` でモデル保存名を組み立て、`save` で `trained_net`・`norm_params`・`experiment_id` を保存します。
4. 続いて `sprintf('%s_predict.mat', experiment_id)` で予測用ファイル名を組み立てます。
5. `predictTireForces` を呼ぶ匿名関数 `predict_fn` を生成し、別ファイルへ保存します。
6. 実験 ID、モデルファイル、予測関数ファイルの保存先を標準出力へ表示します。

## 6. 関数・ローカル関数詳細

### `saveExperimentModel`

- 一意な実験 ID に基づき、学習済みモデルと推論用ハンドルを 2 ファイルへ分けて保存します。
- `predict_fn` 自体は匿名関数であり、このファイル内に追加のローカル関数はありません。
- 関数ハンドルは `trained_net` と `norm_params` を閉包として保持するため、呼び出し時に生データ `X_raw` だけを渡せる形になります。

## 7. 注意点

- `experiment_id` の妥当性検証はしていないため、そのままファイル名に使える文字列であることが前提です。
- `predict_fn` は `predictTireForces` への依存を持つため、読み出し先環境でも当該関数が利用可能である必要があります。
- 上書き防止は「一意な `experiment_id` を与える」運用に依存しており、同じ ID を渡すと同名ファイルが再保存されます。
