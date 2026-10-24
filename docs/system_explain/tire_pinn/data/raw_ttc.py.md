# `tire_pinn/data/raw_ttc.py`

## 1. 役割

raw TTC/TIRF `.dat` を **manifest 対応で解釈し、PINN 学習用の標準列へ正規化** する中核ユーティリティです。  
メタデータ抽出、タイヤ名正規化、試験種別推定、学習スキーマ変換を 1 か所へ集約しています。

## 2. 入出力

- 主な入力:
  - raw TTC `.dat`
  - 任意の `raw_ttc_manifest.json`
- 主な出力:
  - `RawTTCMetadata`
  - 列 `SR, SA, Fz, IA, TSTC, Fx, Fy, Mx, Mz` を持つ `DataFrame`

## 3. 主要データ構造

### `RawTTCMetadata`
- 保持項目:
  - `path`
  - `project_id`
  - `run_number`
  - `tire_name`
  - `tire_key`
  - `test_type`
  - `source_kind`
  - `extra`
- `source_id` プロパティで `project_id:run_number` を返します。

## 4. 主要関数

### `normalize_tire_name(name)`
- TTC 名称をリポジトリ慣例の識別子へ変換します。
- `× -> X`, `.` -> `p`, 空白除去、記号の `_` 化を行います。

### `infer_test_type_from_path(path)`
- パス名から `Db / Cr / Cmb` を推定します。
- フォルダ名やファイル名に `drivebrake`, `cornering`, `combined` があるかを見ます。

### `resolve_manifest_path(manifest_path, data_dir)`
- 明示指定優先、未指定時は `data_dir/raw_ttc_manifest.json` を自動探索します。

### `load_manifest(manifest_path)`
- manifest JSON を読みます。無指定時は空辞書を返します。

### `parse_metadata_line(metadata_line)`
- 先頭行から `Project`, `Run`, `Tire_Name` を正規表現で抽出します。

### `infer_test_type(df, path=None)`
- パス推定を優先し、駄目なら `SR`/`SA` 活動度から `Db/Cr/Cmb/Unknown` を返します。

### `load_raw_tirf_file(path, manifest=None, nrows=None)`
- raw ファイルを読み、列名変換と符号調整を行い、標準学習スキーマへ変換します。
- 戻り値: `(metadata, extracted_df)`
- `source_file`, `source_path`, `source_id`, `source_test_type`, `source_tire_key` も付与します。

### `parse_raw_tirf_metadata(path, manifest=None, sample_rows=200)`
- 全量を使わず、メタデータだけ先読みしたいときの軽量入口です。

## 5. データフローとシステム適合

`TireCatalog` はこのモジュールで raw `.dat` の正体を把握し、  
`TTCDataLoader` は `load_raw_tirf_file()` を使って CSV と同じ訓練スキーマへ統一します。  
つまり raw データ対応の境界層です。

## 6. 副作用と注意点

- manifest 不在時は例外を投げる場合があります。
- `Fz` は絶対値化されます。
- raw ファイル先頭行の品質が低くても、manifest で補完できる設計です。
