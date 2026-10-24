# `tire_fitting/model_selector.py`

## 役割

複数モデルのフィット結果から、縦力 (`fx`) と横力 (`fy`) に使うモデルを選ぶ判定器です。  
自動選択、完全手動、片側だけ手動の hybrid 運用を 1 つの API にまとめます。

## 実行と入出力

- 直接実行するファイルではありません。
- 主な入力:
  - `fitting_results[model][direction][metric]`
  - 任意の `manual_fx`, `manual_fy`
- 主な出力:
  - `(fx_model_name, fy_model_name)` のタプル
- 例外:
  - `ValueError`: manual 指定不足、方向に候補が無い

## 主要クラス

### `SelectionMode`
- `AUTO`, `MANUAL`, `HYBRID` の列挙です。

### `ModelSelector`

#### `select_models(fitting_results, manual_fx=None, manual_fy=None)`
- モードに応じて内部の `_auto_selection`, `_manual_selection`, `_hybrid_selection` を呼び分けます。

#### `_select_best_for_direction(results, direction)`
- 方向ごとの本体判定です。
- 優先順位:
  1. RMSE 最小
  2. 上位 2 モデルの RMSE 差が 5% 未満なら TIC が低い方
  3. R² はログ用で、実装上の最終 tie-break には未使用

#### `_manual_selection(fx, fy)`
- 両方向のモデル名が揃っている時だけ返します。

#### `_hybrid_selection(results, manual_fx, manual_fy)`
- 指定された側は手動、未指定側だけ自動選択します。

## データフロー

1. `ModelFitter` の結果から各モデル・各方向の metric 辞書を受け取ります。
2. `SelectionMode` に従って自動または手動判定へ分岐します。
3. 縦横別の選択結果を後段の validation / exporter / report へ渡します。

## システム内での位置づけ

hybrid パイプラインの **意思決定ロジック** です。  
最適化や計算モデル本体から独立しているため、選択基準だけを差し替えやすい構造になっています。
