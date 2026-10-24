# `matlab/pou_hardpoint_design/test_pou_optimizer.m`

## 1. 目的

POU ハードポイント最適化まわりの主要関数をまとめて試す総合テストスクリプトです。ジオメトリ定義、キングピン導出、運動学、目的関数、制約、パッケージング、左右対称性まで 12 項目を順番に検証します。

## 2. 入出力と依存関係

- 形式: スクリプト
- 入力: 固定の設計変数、操舵角配列、目標係数
- 出力: 各テストの PASS/FAIL と数値、最後の総計
- 依存関係:
  - `define_suspension_geometry`
  - `solve_upright_position_3d`
  - `forward_kinematics`
  - `objective_polynomial_fit`
  - `constraint_rod_misalignment`
  - `validate_packaging`

## 3. 全体処理フロー

1. `geometry` と `optimization` を `addpath` します。
2. `n_pass`, `n_fail` を初期化します。
3. 12 個の `try/catch` テストを順に実行します。
4. 重大失敗とみなす初期テストの一部は `return` で即終了し、後半テストは失敗しても継続します。
5. 最後に合格数と失敗数を表示します。

## 4. テスト項目詳細

1. **Geometry Definition**: 新しい DW フィールド群が `geom` に存在するか確認します。
2. **Kingpin Derivation**: 上下ボールジョイントから導いたキングピン軸が正しいか確認します。
3. **A-arm Rotation Axis**: アーム回転軸と原点が chassis 取付点から正しく導かれているかを見ます。
4. **Zero-Steer Identity**: 0 操舵時の `R_upright` が単位行列、`t_upright` がゼロか確認します。
5. **Upright Rigidity**: 各操舵角でキングピン長が保たれるかを見ます。
6. **A-arm Length Preservation**: 軸からボールジョイントまでの垂直距離が保存されるかを調べます。
7. **Forward Kinematics**: ジャッキアップ、キャンバー、pushrod 長の物理的常識範囲をチェックします。
8. **Objective Function**: 目的関数が有限で非負のスカラーを返すかを確認します。
9. **Constraint Function**: コメント上は 5 制約を期待していますが、実際には `constraint_rod_misalignment` 実装が 8 要素まで返すため、このテストは古い期待値 `length(c) == 5` を使っています。
10. **Packaging Validation**: パッケージング判定が動作するかを確認します。
11. **FL/FR Symmetry**: FL と FR の座標対称性とジャッキアップ曲線の対称性を確認します。
12. **Physical Sanity**: 30 度操舵時のジャッキアップ量とキャンバー変化量の上限を見ます。

## 5. ローカル関数

このファイルにローカル関数はありません。すべてテストブロックとして直列に記述されています。
