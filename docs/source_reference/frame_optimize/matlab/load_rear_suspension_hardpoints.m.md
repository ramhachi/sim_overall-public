# frame_optimize/matlab/load_rear_suspension_hardpoints.m

## 役割

後輪側サスペンションのハードポイントを読み、前輪基準のフレーム座標系へ変換する関数です。後輪サスペンションは現行最適化で **固定境界側の基準点** として重要です。

## 実装のポイント

- JSON の `RL` データを読み、RR 側を左右鏡映で生成する
- `wheelbase_m` を使って「後軸基準」から「前軸基準」へ X 位置を移す
- 8 個のシャシー取付点と、可視化用 6 個のアップライト点を返す

## 出力の順序

`rear_mount_nodes` は `[RL_LF, RR_LF, RL_UF, RR_UF, RL_LR, RR_LR, RL_UR, RR_UR]` の順です。`frameObjectiveFunction` や `run_frame_optimization.m` はこの順番を前提に左右・上下を切り分けています。

## 実装上の意味

前輪側が加振点、後輪側が拘束点という荷重条件を作るため、ここでのインデックス順は最適化結果に直接効きます。
