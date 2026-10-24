# tests/property/test_properties_data.py

## 役割
`TireDataLoader` と `TireDataPreprocessor` に対して、入力分布が広く変わっても崩れてはいけない普遍条件を Hypothesis で検証します。

## 守っている振る舞い
- DriveBrake/Cornering 読み込み後に必須列が必ず残ること。
- 負の `FZ` が正値へ符号反転され、絶対値が保存されること。
- 必須列欠落時に `ValueError` が起き、欠落列名がメッセージに含まれること。
- 前処理フィルタ後の全行が速度・荷重・温度・SA/SR 条件を満たすこと。
- 荷重グループ分けが Low/Medium/High の境界条件を守り、重複行を生まないこと。
- IQR 外れ値除去後の値が計算境界内に必ず収まること。

## 主なフィクスチャ/ヘルパー
- `setup_method()` で `TireDataLoader` と `TireDataPreprocessor` を生成します。
- Hypothesis が `fz_values`、欠落列集合、乱数 seed、IQR 乗数を広く生成します。
- 一部テストは実データ CSV を読み、残りは一時 CSV/DataFrame をその場で構築します。

## アサーションの意味
各 `assert` は「前処理後データは常に仕様域にある」「例外には原因情報が含まれる」という契約を固定します。単なる一例確認ではなく、任意入力でも後段の最適化や学習が壊れないデータ面の安全柵になっています。
