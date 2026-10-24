# tests/unit/test_data_loader.py

## 役割
`TireDataLoader` の基本入出力、必須列検査、`FZ` 符号補正、ファイルエラー処理を例示ベースで検証します。

## 守っている振る舞い
- 実データの DriveBrake/Cornering CSV 読み込みで必須列が揃い、`FZ` が正値化され、空 DataFrame にならないこと。
- 存在しないファイルで `FileNotFoundError` が発生すること。
- 必須列欠落 CSV で `ValueError` が発生すること。
- 負の `FZ` を持つ簡易 CSV が、読み込み後は同じ大きさの正値へ変換されること。

## 主なフィクスチャ/ヘルパー
- `setup_method()` で `TireDataLoader` を生成します。
- 実データファイルと一時 CSV の両方を使い分けています。

## アサーションの意味
このファイルの `assert` は、ローダが最低限のデータ衛生を必ず行うことを固定します。特に `FZ` 正値化と列不足検知は、以後の前処理・フィット計算の前提条件です。
