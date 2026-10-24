# `tire_pinn/data/raw_ttc_parser.py`

## 1. 役割

raw TTC/TIRF `.dat` を読むための **簡易パーサ兼推定器** です。  
`raw_ttc.py` より軽量で、ヘッダ解析・試験種別推定・学習列抽出に絞った補助実装になっています。

## 2. 主な入出力

- 入力:
  - raw TTC `.dat`
- 出力:
  - `RawTTCMetadata`
  - 学習列だけを持つ `DataFrame`

## 3. データ構造

### `RawTTCMetadata`
- `project_id`, `run_number`, `tire_name`, `compound`, `dataset_id`, `display_name`, `test_type` を持ちます。
- `compound` を別保持する点が `raw_ttc.py` 版との違いです。

## 4. 関数

### `normalize_tire_name(tire_name, compound=None)`
- タイヤ名と compound を連結し、識別子へ整形します。

### `_read_header_lines(path)`
- 先頭 3 行の `metadata / columns / units` を返します。

### `infer_test_type_from_path(path)`
- パス名ベースで `Db/Cr/Cmb` を推定します。

### `parse_raw_ttc_metadata(path, sample_rows=200)`
- ヘッダから project/run/tire 名を取り、サンプル行を読んで試験種別も推定します。

### `load_raw_ttc_dataframe(path, nrows=None)`
- 2 行目の列名を使って本体データを `read_csv()` します。

### `infer_test_type(df, path=None)`
- `SR` と `SA` の 75 パーセンタイル活動度から試験種別を決めます。

### `extract_training_columns(df)`
- 生データ列を `SR, SA, Fz, IA, TSTC, Fx, Fy, Mx, Mz` に変換します。
- `TSTC` が無い場合は `TSTI`, `TSTO`, それも無ければ 40℃ を補います。

## 5. システム内での位置付け

- raw TTC 対応の試作/補助レイヤです。
- 現在の主経路は manifest 対応の `raw_ttc.py` 側ですが、このファイルは raw 解釈ロジックの単純版として残っています。

## 6. 副作用と注意点

- 必須列欠落時は `KeyError`/`ValueError` を投げます。
- `dropna().reset_index(drop=True)` により欠測行を落とします。
