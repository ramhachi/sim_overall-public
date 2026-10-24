# `frame_optimize/matlab/generate_onshape_3d_sketch.m`

## 役割

Onshape 用の **3D スケッチ相当 centerline exporter** です。実体は `opPolyline` を並べた FeatureScript snippet を生成し、フレーム要素を wire body として再構築します。

## 何をするか

1. 節点と接続を受け取る
2. SolidWorks exporter と同じ座標変換 `[-X, Z, Y]` を適用する
3. 重複要素を落とす
4. 要素ごとに `opPolyline` を 1 本ずつ出力する
5. Onshape の Feature Studio に貼る `.fs` ファイルを保存する

## この設計にしている理由

- ユーザー要件が **「端面処理を外して 3D スケッチだけ生成したい」** だったため、solid / cope cut 生成は意図的に外しています。
- Onshape では SolidWorks の VBA macro 相当がそのまま存在しないため、まずは **FeatureScript を生成する exporter** に寄せています。
- `opPolyline` の wire body に絞ることで、centerline の確認・手作業の後続モデリング・CAD 側の微調整を分離できます。

## 注意点

- 出力は Feature Studio の先頭 header/import を含まない snippet です。
- つまり **「新規 Feature Studio を作り、既定 header の下へ貼る」** 前提です。
- 断面プロファイル、材料、cut list、トリム、cope cut はこの段階では扱いません。
