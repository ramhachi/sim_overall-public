# frame_optimize/tests/test_frame_obj.m

## 役割

目的関数を 1 回だけ実行して、FEM サーバー起動から結果取得までが通るかを見る MATLAB のスモークテストです。

## していること

- 幾何、材料、カタログ、除外ゾーンを読み込む
- 手動で `config` を組み立てる
- 代表的な設計変数 `x` を 1 組だけ与える
- `beam_fem_solver_tcp_java('start')` 後に `frameObjectiveFunction` を 1 回呼ぶ
- 質量、CG、高さ、剛性を表示して停止する

## 現在の注意点

このテストは「疎通確認」には有用ですが、最新実装とのずれもあります。たとえば現在の `frameObjectiveFunction` は `P_stiff` を返しますが、このスクリプトは `f_stiff` を表示しようとしています。したがって **厳密な自動回帰テストというより手動診断用** と読むのが実装に近いです。
