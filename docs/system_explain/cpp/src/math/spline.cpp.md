# `cpp/src/math/spline.cpp`

## 1. ファイルの目的
1 次元/2 次元スプライン補間クラスのテンプレート実体を生成し、他モジュールからリンク利用できるようにするファイルです。

## 2. 実行上の役割
- 実ロジックは `spline.hpp` にあります。
- この `.cpp` は `CubicSpline` と `BicubicSpline` の `double`/`float` 実体化担当です。

## 3. 依存関係
- `fsae/math/spline.hpp`

## 4. 入力/出力
- 実行入口ではありません。
- 補間クラスのテンプレート実体を生成するだけです。

## 5. 関数・メソッド・クラスの網羅的な説明

### この `.cpp` 自体に独自関数はない
- 内容は `template class ...;` による明示的インスタンス化のみです。

### `CubicSpline<Scalar>`
- `double` 版と `float` 版を生成します。
- `spline.hpp` で定義される主要メソッド:
  - `build(x, y)`: 単調増加 `x` と対応 `y` から自然 3 次スプライン係数を構築
  - `operator()(x)`: 補間値計算
  - `derivative(x)`: 1 階微分計算
  - `find_interval(x)`: 内部の区間探索ヘルパー

### `BicubicSpline<Scalar>`
- `double` 版と `float` 版を生成します。
- `spline.hpp` で定義される主要メソッド:
  - `build(x, y, z)`: 行方向に `CubicSpline` を並べて 2 次元格子データを準備
  - `operator()(x, y)`: 各行で x 補間した後、y 方向を再度 `CubicSpline` で補間

