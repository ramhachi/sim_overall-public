# `frame_optimize/cpp/beam_fem_standalone.cpp`

## 役割

- JSON ファイルを入力に受け、3D Timoshenko 梁 FEM を実行して変位・ねじり剛性・質量を JSON で返す CLI ソルバです。

## 入出力

- **入力ファイル**
  - `nodes`
  - `elements`
  - `bcs`
  - `loads`
- **出力ファイル**
  - `success`
  - `U`
  - `K_torsion`
  - `mass`

## データ構造

### `BeamElement`

- **保持内容**
  - 接続節点
  - 外径、板厚、E、G、密度、形状種別、継手係数
  - 断面積、断面二次モーメント、ねじり定数
  - 長さ、回転行列
- **責務**
  - 1 要素分の FEM 計算に必要な情報をまとめます。

## 関数詳細

### `computeSectionProperties(BeamElement &elem)`

- **入力**
  - 丸管または角管の断面寸法
- **出力**
  - `A`, `Iy`, `Iz`, `J`
- **責務**
  - 断面特性を mm 系で計算します。
- **注意点**
  - `shape` は 1=round, 2=square です。

### `computeRotationMatrix(const Vector3d &p1, const Vector3d &p2)`

- **入力**
  - 両端節点座標
- **出力**
  - 3x3 回転行列
- **責務**
  - 要素軸方向と直交基底を組みます。

### `computeLocalStiffness(const BeamElement &elem)`

- **入力**
  - 要素長、材料、断面、`joint_factor`
- **出力**
  - 12x12 ローカル剛性
- **責務**
  - 軸、ねじり、2 軸曲げを持つ梁要素剛性を構築します。
- **注意点**
  - 単位変換として GPa→Pa、mm²→m²、mm⁴→m⁴ を行います。
  - `joint_factor` は曲げ・ねじりに掛かり、軸剛性には掛かりません。

### `transformToGlobal(const MatrixXd &K_local, const Matrix3d &R)`

- **出力**
  - グローバル座標系の 12x12 剛性
- **責務**
  - ブロック対角変換行列で `K_global = T^T K_local T` を作ります。

### `main(int argc, char* argv[])`

- **入力**
  - `input.json`, `output.json`
- **責務**
  - 全処理の制御を行います。
- **処理内容**
  - JSON 読込
  - 節点・要素・拘束・荷重の配列化
  - 全体剛性組立
  - 到達不能節点の BFS 検出と拘束追加
  - 疎行列求解
  - ねじり剛性と質量計算
  - 結果 JSON 出力
- **注意点**
  - 入出力の節点番号と DOF は MATLAB 互換の 1 始まりです。

## 実装上の要点

- 到達不能節点には拘束を追加し、特異行列を避けます。
- ねじり剛性は `Rx` 由来または 2 点カップル荷重由来の角度から求めます。
- 境界条件は大ペナルティ法で適用しています。

## 前提・注意点

- 小変形線形解析です。
- 部材内部力や応力出力は主目的ではなく、変位・剛性・質量の評価に寄っています。
