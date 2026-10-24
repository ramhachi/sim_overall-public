# `matlab/apply_named_tire_model_selection.m`

## 1. ファイルの目的

車両パラメータ構造体 `base` に対して、タイヤモデル種別と必要メタデータを固定設定として適用する関数群です。  
TMeasy / Magic Formula / PINN の切替と、PINN モデル探索・妥当性確認・代表 μ 推定をこの 1 ファイルで行います。

## 2. 主な入出力または依存関係

- 主入力:
  - `base`
  - `tire_model_family`
  - `tire_model_name`
  - `project_root`
- 主出力:
  - `base`
  - `selected_model`
  - `pinn_catalog`
- 依存関係:
  - `output/tire_models/**/manifest.json`
  - `output/tire_models/**/test_vectors.json`
  - `data/tiredata/**/*.dat`
  - MATLAB の `jsondecode`, `fileread`, `dir`

## 3. 処理の流れ / アルゴリズム

1. 入力未指定時の `base` と `project_root` を補完します。
2. `scan_deployable_pinn_catalog()` で deployable な PINN モデル候補を列挙します。
3. `tire_model_family` に応じて TMeasy / Magic Formula / PINN の設定を分岐します。
4. PINN の場合は名前一致候補を検索し、なければ manifest を直接見に行って `build_catalog_entry()` で 1 件組み立てます。
5. `estimate_planner_mu()` が `test_vectors.json` から 95 パーセンタイル相当の `mu_x_max`, `mu_y_max` を推定します。
6. 最後に最低限の μ フォールバック値を補完して返します。

## 4. ファイル内の全関数

### `apply_named_tire_model_selection(base, tire_model_family, tire_model_name, project_root)`

- メイン関数です。
- `tmeasy` では `model_type` だけ残し、PINN/MF 専用フィールドを削除します。
- `magic_formula` では `model_dir` を `/workspace/cpp_export/magic_formula_params.txt` に固定します。
- `pinn` では manifest 由来の `model_dir`, `model_manifest`, `model_name`, `mu_x_max`, `mu_y_max` を設定します。

### `scan_deployable_pinn_catalog(project_root)`

- `output/tire_models` 配下を走査し、`manifest.json` を持つディレクトリだけを候補にします。
- `deployable=true` か、`dataset.identifier` とタイヤ名が一致するか、`data/tiredata` 側に学習対象タイヤが存在するかを確認します。
- 条件を満たした候補を `build_catalog_entry()` で構造体化し、名前順に整列して返します。

### `find_catalog_entry(models, tire_name)`

- カタログ中から完全一致する 1 件を返す単純検索ヘルパーです。

### `build_catalog_entry(project_root, tire_name, manifest_path, manifest)`

- manifest を読み、deployable か確認した上で 1 タイヤモデル分のメタデータ struct を構築します。
- `/workspace/output/tire_models/<name>` 形式のコンテナパスも同時に生成します。
- `estimate_planner_mu()` を呼び、プランナ用の代表 μ も埋め込みます。

### `estimate_planner_mu(test_vectors_path)`

- `test_vectors.json` の入出力行列から `Fx/Fz`, `Fy/Fz` を計算し、代表摩擦係数上限を推定します。
- 荷重 `Fz > 50N` の有限値だけを使い、極端値ではなく `robust_positive_quantile()` の 95% 点を採用します。

### `json_array_to_matrix(value)`

- JSON デコード結果が数値行列でも cell 配列でも処理できるよう、常に数値行列へ正規化します。

### `robust_positive_quantile(samples, fallback_value)`

- 正の有限値だけを抽出してソートし、95% 点を返します。
- 範囲は 0.5〜4.0 にクリップされます。

### `remove_optional_tire_fields(tire, field_names)`

- 不要なオプションフィールドを安全に削除するヘルパーです。

### `scan_trainable_tire_datasets(project_root)`

- `data/tiredata/**/*.dat` の 1 行目を読み、`Tire_Name=` または `Project <番号>` からデータセット識別子を作ります。
- 学習対象として存在するタイヤ名の一覧を返します。

### `normalize_tire_name(raw_name)`

- 空白、カンマ、ピリオド、乗算記号を置換し、データセット名をディレクトリ名互換の識別子へ正規化します。

## 5. 注意点・前提条件

- PINN モデル探索は `manifest.json` と `test_vectors.json` の形式に依存します。
- `magic_formula` のパラメータファイル場所は固定文字列なので、C++ 側の配置変更には追従しません。
- `scan_trainable_tire_datasets()` は `.dat` 先頭行の表記に依存しており、ヘッダ形式が変わると候補除外が起きます。
- `pinn` で `tire_model_name` が空だと即エラーになります。
