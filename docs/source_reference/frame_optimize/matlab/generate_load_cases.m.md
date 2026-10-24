# frame_optimize/matlab/generate_load_cases.m

## 役割

frame objective 用の **荷重条件と境界条件** を作る helper です。主経路では「後輪側固定・前輪側加振」の torsional load case を返します。

## 主な入力

- `nodes`
- `total_mass_kg`
- `g`
- `node_config`

## 実装上の注意

- `node_config` には front / rear suspension mount の node index 群が必要です。
- canonical flow では rear hardpoints 不足を許さず、この関数に入る前に止めます。
