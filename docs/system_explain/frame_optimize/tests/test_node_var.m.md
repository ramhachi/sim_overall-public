# test_node_var.m
## ファイル目的
- `generate_parametric_nodes` の上下限近傍入力でノード数と主要節点変位を比較する。

## 実行フロー
- 実 geometry と suspension JSON を読む。
- 最小側パラメータと最大側パラメータで 2 回 `generate_parametric_nodes` を呼ぶ。
- ノード数、Front Hoop top、Side Impact top、基準 23 ノードの最大差分を表示する。

## テスト詳細
### ケース 1: 最小境界
- シグネチャ: `nodes1 = generate_parametric_nodes(..., [0.50,0.25,0.35,0.25], [0,0], [0.2,...])`
- 入力: 最小側 shape/node 変数
- 出力: `nodes1`
- 責務: 下限側形状の生成確認

### ケース 2: 最大境界
- シグネチャ: `nodes2 = generate_parametric_nodes(..., [0.80,0.45,0.55,0.75], [1,1], [0.8,...])`
- 入力: 最大側 shape/node 変数
- 出力: `nodes2`
- 責務: 上限側形状の生成確認

### 比較セクション
- シグネチャ: `diff = abs(nodes1(1:23,:) - nodes2(1:23,:))`
- 入力: `nodes1`, `nodes2`
- 出力: `max_diff`
- 責務: 基準骨格がどれだけ動くかの確認
- 前提条件・注意事項:
  - assert は無く、目視確認用スクリプトに近い
