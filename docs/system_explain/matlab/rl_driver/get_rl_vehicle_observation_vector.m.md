# `matlab/rl_driver/get_rl_vehicle_observation_vector.m`

## 1. 役割

`get_rl_vehicle_observation_vector` は、車両パラメータ構造体から RL 観測へ埋め込む 11 要素の「車両固有特徴量」を取り出す関数です。  
`FSAEEnvironment.normalizeObservation()` がサーバー由来の 34 要素へ後付けで連結するデータをここで作ります。

## 2. `get_rl_vehicle_observation_vector(params)` 詳細

返り値 `obs_vehicle` は列ベクトルで、順序は以下の通りです。

1. `params.mass.m_total`
2. `params.aero.CL_A`
3. `params.aero.CD_A`
4. `params.tire.mu_y_max`
5. `params.tire.mu_x_max`
6. `params.suspension.k_spring(1)` ＝前輪側ばね定数代表値
7. `params.suspension.k_arb_f`
8. `params.steering.stiffness`
9. `params.suspension.K_chassis`
10. `params.suspension.C_chassis`
11. `params.suspension.I_twist`

## 3. 観測契約上の意味

この 11 要素は `get_rl_observation_bounds()` の 35〜45 次元と 1 対 1 に対応しています。  
そのため、順序や意味を変えると正規化境界・学習済みエージェント・export 先の契約が同時に崩れます。

## 4. 実装上の要点

- 動的状態ではなく、エピソード中に固定の車両条件を観測へ埋め込むための関数です。  
- Domain Randomization を有効にしたとき、エージェントが「今どの車両条件で走っているか」を知る入口になります。  
- 前輪ばねだけ `k_spring(1)` を代表値として使っており、左右差や後輪値はここでは含めません。
