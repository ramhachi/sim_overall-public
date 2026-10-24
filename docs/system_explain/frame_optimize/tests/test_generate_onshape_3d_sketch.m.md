# `frame_optimize/tests/test_generate_onshape_3d_sketch.m`

## 役割

Onshape 向け 3D sketch exporter の回帰テストです。Onshape API を叩くのではなく、**生成された FeatureScript テキストの構造**を確認します。

## 何を確認するか

1. 合成フレームから `.fs` を生成できる
2. `defineFeature` と `opPolyline` を含む
3. `extrude` / `boolean` を含まず、centerline-only 方針を崩していない
4. 座標変換が `[-X, Z, Y]` になっている
5. 出力セグメント数が接続数と整合する

## システム上の意味

この exporter は Onshape 側実行より前に **生成内容の契約**を守っているかが重要です。特に「3D 中心線だけを出し、端面加工へ踏み込まない」という今回のスコープを固定する役割があります。
