# `tire_pinn/data/tire_catalog.py`

## 1. 役割

CSV と raw `.dat` を横断して、利用可能なタイヤデータセットを **一覧化・被覆判定・選択支援** するカタログです。

## 2. 入出力

- 生成入力:
  - `data_dir/csv/*.csv`
  - `data_dir` 配下の raw `.dat`
  - 任意 manifest
- 主な出力:
  - `self.datasets`
  - `list_tire_models()`, `get_complete_datasets()` などの問い合わせ結果
- 直接実行時:
  - 標準出力へ一覧表を表示します。

## 3. 主要フロー

1. `__init__()` で manifest を解決し、`_scan_datasets()` を実行します。
2. CSV はファイル名 `Db|Cr|Cmb_<identifier>_RunN.csv` から分類します。
3. raw `.dat` は `parse_raw_tirf_metadata()` で識別子と test type を推定します。
4. `_register_file()` が `has_db/has_cr/has_cmb` と件数、ソース種別を更新します。
5. 最後に `Db + Cr` が揃っているものを `deployable=True` とします。

## 4. メソッド

### `_empty_entry(identifier, display_name)`
- データセット辞書の初期形を返します。
- メーカー名やサイズ推定もここで行います。

### `_register_file(entry, test_type, path, source_format, ...)`
- `Db/Cr/Cmb` 別の件数とファイル一覧を更新します。

### `_scan_datasets()`
- CSV と raw データを走査し、総合カタログを構築する本体です。

### `list_tire_models()`
- 識別子一覧を返します。

### `get_tire_info(tire_name)` / `get_dataset_entry(identifier)`
- 対象データセットの詳細辞書を返します。
- 後者は未登録時に `KeyError` を投げます。

### `filter_by_manufacturer(manufacturer)` / `filter_by_size(size_pattern)`
- カタログ検索補助です。

### `get_complete_datasets()` / `get_deployable_datasets()`
- Db/Cr が揃う識別子だけを返します。

### `print_catalog()`
- メーカー単位で整形表示します。

## 5. システム内の位置付け

`main_pinn_train.py` が「何を学習対象にするか」を決める前段に置かれる索引です。  
単なる一覧ではなく、**学習可能か・デプロイ候補か** を判定する入口でもあります。

## 6. 副作用

- 初期化時にファイルシステムを再帰走査します。
- `__main__` 実行時は一覧を標準出力へ出します。
