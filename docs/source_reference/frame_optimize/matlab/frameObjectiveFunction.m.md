# frame_optimize/matlab/frameObjectiveFunction.m

## 役割

最適化 1 回分の評価を行う、このサブシステムの中心関数です。設計変数 `x` から節点と部材を作り、FEM を解き、質量・CG・剛性・干渉をまとめて 1 つの評価値に変換します。

## 実装の流れ

1. `x` をパイプ選択、形状、距離しきい値、節点 ON/OFF、節点比率に分解する
2. `generate_parametric_nodes` で前後サスペンションを含む節点群を作る
3. `generate_adaptive_topology` で接続候補を作る
4. 各部材へカタログ断面と材料物性を割り当てる
5. 部材長から総質量と CG 高さを再計算する
6. `generate_load_cases` で「後輪固定・前輪加振」の荷重条件を作る
7. `beam_fem_solver_tcp_java` 経由で C++ FEM を解く
8. 除外ゾーン衝突、CG 超過、部材数・総延長過多をペナルティ化する
9. `W_mass * f_mass + W_cg * f_cg + P_stiff + penalties` を返す

## 返す詳細値

`results` には `mass`、`K_torsion`、`cg_z`、`P_stiff`、`P_mass_target`、各種 penalty、生成節点などが入ります。最終サマリーやデバッグ表示はこの構造体を使います。

## 実装上の注意

- 引数名に `nodes_base`、`bcs`、`loads` がありますが、現在はほぼ互換用で、実際には関数内で再生成しています。
- `checkFrameCompliance.m` はここでは呼ばれておらず、現行目的関数は材料制約・アルミ厚み・除外ゾーンなどを個別に罰則化する方式です。
- 除外ゾーンに当たった設計は `Inf` を返して早めに淘汰します。
- canonical flow では `config.require_rear_suspension_hardpoints` と `config.require_total_mass_kg` を使って、rear hardpoints や車両質量が欠けた状態での暗黙フォールバックを止めます。
- 疎性ペナルティの閾値は `config.sparsity_max_total_length_m` と `config.sparsity_max_elements` で上書きできます。
