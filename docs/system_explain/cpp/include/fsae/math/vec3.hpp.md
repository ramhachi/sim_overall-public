# vec3.hpp

## ファイルの目的
- 軽量な 3 次元ベクトル型 `Vec3` を提供します。

## 実行上の役割
- Eigen を使わずに小さなベクトル演算を行いたい箇所向けの基礎部品です。
- `alignas(32)` により SIMD 友好的なメモリアラインメントを持ちます。

## 依存関係
- 外部: `<cmath>`, `<array>`
- リポジトリ内依存: なし

## 入力/出力
- 入力: スカラー値 3 成分、他の `Vec3`
- 出力: `Vec3<Scalar>`、内積値、ノルム値

## 関数・メソッド・クラスの網羅的な説明
- `template<typename Scalar> struct Vec3`
  - メンバは `x`, `y`, `z` の 3 成分です。
  - `constexpr Vec3()` はゼロ初期化します。
  - `constexpr Vec3(Scalar x_, Scalar y_, Scalar z_)` は 3 成分指定コンストラクタです。
  - `operator[](int)` / `operator[](int) const` は `x,y,z` を配列風に参照します。
  - `operator+`, `operator-` は要素ごとの加減算です。
  - `operator*(Scalar)`, `operator/(Scalar)` はスカラー倍・除算です。
  - `operator+=`, `operator-=`, `operator*=` は破壊的更新版です。
  - `dot(const Vec3&)` は内積を返します。
  - `cross(const Vec3&)` は外積ベクトルを返します。
  - `norm()` はユークリッドノルム、`norm_sq()` は二乗ノルムです。
  - `normalized()` はノルムが `1e-12` より大きいとき正規化し、ほぼゼロならゼロベクトルを返します。
  - `static zero()/unit_x()/unit_y()/unit_z()` は代表的な定数ベクトル生成です。
- `template<typename Scalar> Vec3<Scalar> operator*(Scalar s, const Vec3<Scalar>& v)`
  - 左スカラー乗算を有効にする自由関数です。
- `using Vec3d = Vec3<double>`
  - `double` 精度エイリアスです。
- `using Vec3f = Vec3<float>`
  - `float` 精度エイリアスです。
