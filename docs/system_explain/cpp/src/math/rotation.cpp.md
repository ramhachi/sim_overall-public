# `cpp/src/math/rotation.cpp`

## 1. ファイルの目的
回転変換テンプレートの明示的インスタンス化を行い、`double` と `float` 版の実体をリンク可能にするファイルです。

## 2. 実行上の役割
- テンプレート実装本体は `rotation.hpp` にあります。
- この `.cpp` はオブジェクトコード生成責務のみを持ち、回転計算ユーティリティを複数翻訳単位から再利用できるようにします。

## 3. 依存関係
- `fsae/math/rotation.hpp`
- `Eigen`

## 4. 入力/出力
- 実行ファイル入口ではありません。
- 生成物はリンク対象のテンプレート実体です。

## 5. 関数・メソッド・クラスの網羅的な説明

### この `.cpp` 自体に独自関数はない
- 定義されているのはテンプレート明示的インスタンス化のみです。

### `EulerAngles<Scalar>`（`rotation.hpp` 側）
- ロール `phi`、ピッチ `theta`、ヨー `psi` を持つ構造体です。
- 本ファイルではこの型を使うテンプレート関数群の実体化対象になります。

### `euler_to_rotation_matrix(const EulerAngles<Scalar>&)`
- `double` 版と `float` 版を明示的に生成します。
- Z-Y-X オイラー角から、慣性座標系→車両座標系の 3x3 回転行列を返します。

### `angular_velocity_to_euler_rate(const EulerAngles<Scalar>&)`
- `double` 版と `float` 版を生成します。
- `[p,q,r]` から `[phi_dot,theta_dot,psi_dot]` を得る変換行列を返します。

### `gravity_in_body(const EulerAngles<Scalar>&, Scalar g)`
- `double` 版と `float` 版を生成します。
- SAE J670e で下向き正の重力ベクトルを車体系へ写します。

