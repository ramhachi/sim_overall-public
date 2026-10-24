# `matlab/pou_hardpoint_design/run_all_wheels.m`

## 1. 目的

POU ハードポイント最適化の一括実行関数です。既定では前輪 FL/FR を対象に順次最適化し、CSV・JSON・レポート出力や車両 JSON 更新をまとめて行えます。

## 2. 入出力と依存関係

- 入力:
  - `params_json_file`: 車両 JSON パス
  - `options`:
    - `wheels`（既定 `[0, 1]`）
    - `n_multistart`（既定 `100`）
    - `delta_range_deg`, `n_delta_points`
    - `output_dir`（既定 `output`）
    - `export_csv`, `export_json`, `export_report`, `update_params`, `verbose`
- 出力:
  - `all_results`: `FL`, `FR`, `RL`, `RR` などのフィールドを持つ構造体
- 依存関係:
  - `main_pou_hardpoint_optimizer`
  - `load_vehicle_artifact`
  - `define_suspension_geometry`
  - `export_hardpoints_csv`
  - `export_hardpoints_json`
  - `export_summary_report`
  - `update_vehicle_params_json`

## 3. 全体処理フロー

1. 既定オプションを補完し、親・子フォルダを `addpath` します。
2. 出力ディレクトリを必要に応じて `mkdir` します。
3. `json_output = <output_dir>/pou_hardpoints.json` を決めます。
4. `options.wheels` を順に回し、各ホイール用 `wheel_opts` を組みます。
5. FR (`wi == 1`) かつ FL 成功済みなら、FL 結果を Y 反転して FR 用 `override_x` を作り、左右対称を強制します。
6. `main_pou_hardpoint_optimizer` を実行し、成功したら `geom` を付加して `all_results.(wheel_name)` に格納します。
7. オプションに応じて CSV/JSON/レポート/車両 JSON 更新を行います。
8. 失敗時は `error` フィールドだけを持つ構造体を格納します。
9. 最後に全体所要時間と、ホイール別サマリー表を表示します。

## 4. 関数詳細

### `run_all_wheels(params_json_file, options)`

- **既定対象**  
  コメントどおり既定では前輪のみです。後輪は `options.wheels` に明示しない限り処理しません。

- **左右対称処理**  
  FR では FL の `M_pivot_chassis` と `bellcrank_pivot` の Y 成分だけを反転し、FR 幾何の 0 操舵変換から `M_pivot_upright_fr` を逆算して `override_x` に入れます。そのため FR は通常の最適化ではなく、FL ミラー形状の評価モードになります。

- **エクスポート制御**  
  CSV はホイール別、JSON は `pou_hardpoints.json` に追記更新、レポートはホイール別 txt です。`update_params=true` のときだけ元の車両 artifact を更新します。

- **エラーハンドリング**  
  各輪ごとに `try/catch` されているため、1 輪失敗しても残り輪は継続します。

- **ローカル関数**  
  このファイルにローカル関数はありません。
