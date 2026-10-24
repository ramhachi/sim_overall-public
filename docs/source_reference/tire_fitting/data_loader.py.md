# tire_fitting/data_loader.py

## 役割
TTC の CSV/生 `.dat` を読み、DriveBrake と Cornering を共通 DataFrame へ整形するローダです。

## 主なクラス・関数
- `TireDataLoader`
- `_resolve_candidate_paths()`: 旧命名・生データを含め候補ファイルを探索
- `_resolve_raw_dat_paths()`: `tire_pinn.data.raw_ttc.parse_raw_tirf_metadata` を使って `.dat` を特定
- `_read_ttc_file()`: 構造化 CSV と TIRF 形式を吸収
- `_prepare_frame()`: 必須列/任意列補完と `FZ` 正値化
- `load_drivebrake_data()`, `load_cornering_data()`

## 背景
同じタイヤでも CSV 名と生データ名が揺れるため、命名正規化と manifest 風メタデータ解析が必要です。本ファイルはその吸収層です。

## 使用ライブラリ
- `pandas`, `numpy`, `pathlib`, `re`, `logging`
- `tire_pinn.data.raw_ttc`

## 入出力
- 入力: CSV または raw TTC `.dat`
- 出力: 標準列を持つ `DataFrame`

## パイプライン上の位置
タイヤ同定パイプラインの最初の実データ取り込み担当です。
