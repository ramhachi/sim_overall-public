# frame_optimize/matlab/generate_adaptive_topology.m

## 役割

生成済み節点から **フレーム部材候補の接続グラフ**を作る helper です。前方・側方・後方の距離しきい値と group 情報に基づいて connectivity を返します。

## 出力の中心

- `connectivity`
- `group_indices`
- `group_names`
- `base_elements`

## 実装上の注意

- `frameObjectiveFunction.m` と canonical API の両方で使われます。
- ここで決まる `group_indices` が、どの pipe catalog 変数をどの部材へ割り当てるかの土台になります。
