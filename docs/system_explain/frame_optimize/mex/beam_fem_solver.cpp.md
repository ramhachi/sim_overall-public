# `frame_optimize/mex/beam_fem_solver.cpp`

## 役割

- MATLAB から直接呼べる MEX 版 3D 梁 FEM ソルバです。
- `nodes`, `elements`, `bcs`, `loads` を MATLAB 配列で受け、変位・ねじり剛性・質量を struct で返します。

## MATLAB インターフェース

- 呼び出し形:
  - `result = beam_fem_solver(nodes, elements, bcs, loads, options);`

## データ構造

### `BeamElement`

- **保持内容**
  - 接続節点
  - 断面寸法
  - 材料定数
  - `joint_factor`
  - 断面特性
  - 長さ・回転行列

## 関数詳細

### `computeSectionProperties(BeamElement &elem)`

- 丸管/角管の断面積、断面二次モーメント、ねじり定数を計算します。
- 不正 `shape` は `mexErrMsgIdAndTxt` で MATLAB 例外にします。

### `computeRotationMatrix(const Vector3d &p1, const Vector3d &p2)`

- 要素軸に沿った局所基底を作成します。

### `computeLocalStiffness(const BeamElement &elem)`

- **出力**
  - 12x12 ローカル剛性
- **責務**
  - スタンドアロン版と同様、軸・曲げ・ねじりを含む剛性を返します。

### `transformToGlobal(const MatrixXd &K_local, const Matrix3d &R)`

- 座標変換でグローバル剛性へ写像します。

### `mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])`

- **入力**
  - `prhs[0]`: `nodes` Nx3
  - `prhs[1]`: `elements` Mx9
  - `prhs[2]`: `bcs` Bx3
  - `prhs[3]`: `loads` Lx3
  - `prhs[4]`: `options`（未使用）
- **出力**
  - `result.U`
  - `result.K_torsion`
  - `result.mass`
  - `result.success`
- **責務**
  - MATLAB 配列検証、C++ 形式への変換、FEM 解法、結果 struct 生成をまとめます。
- **主要ロジック**
  - 入力サイズと型の検証
  - 1 始まり index の 0 始まり化
  - 全体剛性組立
  - 浮遊節点検出と拘束追加
  - 疎行列ソルバ実行
  - ねじり剛性・質量算出
- **注意点**
  - コメントには `element_forces` が記載されていますが、実装上の主出力は `U`, `K_torsion`, `mass`, `success` です。

## 前提・注意点

- MATLAB 配列は列優先なので、入力走査時の添字計算は C++ 側で明示的に行っています。
- 拘束適用はペナルティ法で、極端な条件数悪化に注意が必要です。
