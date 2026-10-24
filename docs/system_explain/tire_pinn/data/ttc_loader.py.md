# `tire_pinn/data/ttc_loader.py`

## 1. 役割

TTC の CSV / raw `.dat` をまとめて読み、**学習用 DataFrame・正規化係数・PyTorch DataLoader** へ変換するデータ入口です。

## 2. 入出力

- 主な入力:
  - 明示 CSV パス、または `tire_name/dataset_id + data_dir`
  - manifest
  - `max_samples_total`, `sampling_seed`
- 主な出力:
  - `self.df`
  - `component_mask`
  - 正規化係数
  - `TensorDataset`, `DataLoader`

## 3. 主要処理

1. `__init__()` で direct CSV モードか `TireCatalog` モードかを決めます。
2. `_load_and_merge()` が Db/Cr ファイル群を読み込み、結合・外れ値除去・必要なら層化サブサンプリングを行います。
3. `source_test_type` から `component_mask` を作り、Fx/Fy/Mx/Mz の有効成分を保持します。
4. `normalize=True` なら平均と標準偏差を計算します。
5. `split_data()` / `create_dataloaders()` で訓練・検証・テストへ分割します。

## 4. メソッド

### `_load_paths(paths)`
- `.csv` は `_load_csv_file()`、`.dat` は `load_raw_tirf_file()` へ振り分けます。

### `_load_csv_file(path)`
- TTC CSV の列名を学習スキーマへ写像し、ソース情報列も追加します。

### `_subsample_dataframe(df)`
- `source_test_type` 比率を保つ層化サンプリングを行います。

### `_build_component_mask_array(source_types)`
- `Db` では Fy を無効、`Cr` では Fx を無効、`Cmb` では全成分有効というマスクを返します。

### `_compute_normalization_params()`
- 入力 `[SR, SA, Fz, IA, TSTC]` と出力 `[Fx, Fy, Mx, Mz]` の平均/標準偏差を計算します。

### `get_normalization_params()`
- 上記統計を `torch.Tensor` 辞書で返します。

### `get_input_bounds()`
- 各入力列の最小/最大を返します。

### `split_data(train_ratio, val_ratio, test_ratio, random_seed)`
- ランダムシャッフル後に `TensorDataset` を 3 分割します。
- 戻り値各要素は `(X, y, component_mask)` を持ちます。

### `create_dataloaders(...)`
- `split_data()` の結果を `DataLoader` 化します。

## 5. データフローとシステム適合

このファイルが **実験データ世界** と **PyTorch 学習世界** の橋渡しです。  
`PINNTrainer` はここから得た正規化係数・DataLoader・成分マスクに依存して学習します。

## 6. 副作用と注意点

- 読み込み時に件数、除去サンプル数、正規化係数を print します。
- `|SR|>1.5`, `Fz<=0`, 異常 `TSTC` を除外します。
- Db か Cr のどちらかが欠けると警告を出します。
