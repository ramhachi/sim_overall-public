# `frame_optimize/tests/test_generate_onshape_3d_sketch.m`

## ファイル目的

- `generate_onshape_3d_sketch` が **FeatureScript を生成できること** と、centerline-only 方針を守っていることを静的に検証する。

## 実行フロー

1. `tempdir` 配下に一時出力先を作る
2. 合成フレーム節点と接続を構築する
3. `generate_onshape_3d_sketch(...)` を呼ぶ
4. 出力 `.fs` を読み、feature wrapper / 座標変換 / `opPolyline` 数を検証する
5. 一時ディレクトリを削除する

## テスト詳細

### `test_generate_onshape_3d_sketch()`
- シグネチャ: `function test_generate_onshape_3d_sketch()`
- 責務:
  - Onshape exporter の静的スモークテスト

### 主な確認項目
- 出力ファイルが存在し、十分なサイズを持つ
- `defineFeature(...)` と `opPolyline(...)` が含まれる
- `extrude` / `boolean` が含まれず、solid/cope を作らない
- Node 1 の座標が `[-X, Z, Y]` に変換されている
- `opPolyline` 本数が unique な接続数と一致する
