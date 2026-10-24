# tests/unit/test_tire_pinn_data_sources.py

## 役割
PINN 用タイヤデータ源の自動発見、マニフェスト解釈、教師あり成分マスク生成、損失/R² のマスク尊重を検証します。

## 守っている振る舞い
- `TireCatalog` が Round9 の nested `.dat` をディレクトリ種別から Db/Cr として正しく検出すること。
- Round8 の project-only ヘッダでも manifest を使って同一タイヤの Cornering 群を自動発見できること。
- Round6 では DriveBrake と Cornering を統合し、deployable 判定や件数集計が正しいこと。
- `TTCDataLoader` が `coverage`、`source_tire_key`、`source_test_type` を正しく構成すること。
- `component_mask` が Db では Fx を、Cr では Fy を教師あり対象にすること。
- `DataLoss` と `R2Score` が未教師成分を無視し、監督対象だけで評価すること。

## 主なフィクスチャ/ヘルパー
- `_write_raw_dat()` と `_write_project_only_raw_dat()` が TTC 風 `.dat` テストデータを生成します。
- `tmp_path` を使って各ラウンド相当のディレクトリ構造と `raw_ttc_manifest.json` を再現します。

## アサーションの意味
このファイルのアサーションは、データ発見ロジックと学習監督マスクの両方を固定します。とくに `component_mask` と損失計算の検証は、DriveBrake に Fy 誤差、Cornering に Fx 誤差を誤って課す事故を防ぐ要点です。
