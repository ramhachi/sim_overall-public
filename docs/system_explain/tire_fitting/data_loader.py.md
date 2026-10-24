# `tire_fitting/data_loader.py`

## 役割

TTC 由来の DriveBrake / Cornering データを読み込むローダです。  
CSV と `.dat` の両系統を扱い、列不足の補完、`FZ` の絶対値化、旧命名 CSV から新命名ファイル群への解決まで担当します。

## 実行と入出力

- 直接実行するファイルではありません。
- 主な入力:
  - DriveBrake / Cornering のファイルパス
  - `data/tiredata/csv` 配下の新形式 CSV
  - `data/tiredata` 配下の raw `.dat`
- 主な出力:
  - 必須列を満たす `pandas.DataFrame`
  - `FZ` を正値化したデータ
- 例外:
  - `FileNotFoundError`: 候補ファイルが見つからない
  - `ValueError`: 必須列不足や読込失敗

## 主要クラス

### `TireDataLoader`

#### `DRIVEBRAKE_COLUMNS` / `CORNERING_COLUMNS`
- TTC データとして要求する列定義です。
- `OPTIONAL_COLUMNS` で新形式 CSV / raw `.dat` 向けの補完列も持ちます。

#### `_normalize_legacy_tire_name(raw_name)`
- 旧ファイル名のタイヤ名を新形式識別子へ寄せます。
- 空白除去、`.`→`p`、`×` / `x`→`X`、`-R`→`_R` などを実施します。

#### `_resolve_candidate_paths(filepath, prefix)`
- まず指定パスの存在を確認します。
- 見つからなければ `DriveBrake-...` / `Cornering-...` の旧名から新形式 CSV 群を探索し、さらに raw `.dat` も検索します。

#### `_resolve_raw_dat_paths(data_dir, identifier, prefix)`
- `tire_pinn.data.raw_ttc.parse_raw_tirf_metadata()` で `.dat` のメタデータを読み、タイヤ識別子と試験種別で一致するファイルを集めます。

#### `_read_ttc_file(filepath)`
- `.dat` は空白区切り、CSV は先頭数行の形で structured / legacy を判定して読み方を切り替えます。

#### `_prepare_frame(df, required_columns, optional_columns)`
- 任意列のデフォルト補完、必須列検査、`FZ` の数値化と絶対値化を行います。

#### `load_drivebrake_data(filepath)`
- 縦力用データを複数ファイル結合込みで読み込みます。
- 戻り値は `SR`, `FX`, `FZ`, `SA` などを含む DataFrame です。

#### `load_cornering_data(filepath)`
- 横力用データを複数ファイル結合込みで読み込みます。
- 戻り値は `SA`, `IA`, `FY`, `FZ`, `MZ`, `CP` などを含む DataFrame です。

## データフロー

1. 入力パスから直接読込または旧名→新名 / raw `.dat` 解決を行います。
2. 各ファイルを TTC 形式に応じて `read_csv` します。
3. 必須列検証と任意列補完を通し、`FZ` を正値へ統一します。
4. 複数ファイルがあれば `concat` して上流へ返します。

## システム内での位置づけ

フィッティング系の入口です。  
`main.py`、`main_hybrid.py`、`optimizer.ModelFitter`、`validate_predictions.py` が共通で依存し、データ形式差をこの層で吸収しています。
