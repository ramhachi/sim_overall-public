# lugre.hpp

## ファイルの目的
- スティックスリップを含む LuGre 摩擦モデルを提供します。

## 実行上の役割
- サスペンションや接触部の摩擦を、毛たわみ状態 `z` を持つ動的モデルとして計算します。
- 高剛性時にはサブステップ積分へ自動切替でき、1 ms 系のメイン積分でも破綻しにくい設計です。

## 依存関係
- 外部: `<cmath>`
- リポジトリ内依存: なし

## 入力/出力
- 入力: `v`, `z`, `dt`, `LuGreParams`
- 出力: 摩擦力 `force`、毛たわみ微分 `dz_dt`

## 関数・メソッド・クラスの網羅的な説明
- `template<typename Scalar> struct LuGreParams`
  - `sigma_0`, `sigma_1`, `sigma_2` は毛剛性・毛減衰・粘性摩擦です。
  - `Fc`, `Fs`, `vs` はクーロン摩擦、静止摩擦、ストライベック速度です。
- `stribeck_function(const LuGreParams<Scalar>& params, Scalar v)`
  - 速度依存の有効摩擦限界 `g(v)` を返します。
- `compute_lugre_force(const LuGreParams<Scalar>& params, Scalar v, Scalar z, Scalar dz_dt)`
  - `sigma_0*z + sigma_1*dz_dt + sigma_2*v` を返します。
- `compute_lugre_dz(const LuGreParams<Scalar>& params, Scalar v, Scalar z)`
  - 毛たわみ微分 `v - (sigma_0/g(v))*|v|*z` を返します。
- `template<typename Scalar> struct LuGreOutput`
  - 力と状態微分の組です。
- `compute_lugre(const LuGreParams<Scalar>& params, Scalar v, Scalar z)`
  - `dz_dt` を先に求め、その値を使って力を返す標準版です。
- `compute_lugre_substep(const LuGreParams<Scalar>& params, Scalar v, Scalar z_initial, Scalar dt, int substeps = 10)`
  - 簡易 Euler 法で `z` をサブステップ積分し、最終状態での力と微分を返します。
  - 高 `sigma_0` 条件での数値安定化が主目的です。
- `compute_lugre_adaptive(const LuGreParams<Scalar>& params, Scalar v, Scalar z, Scalar dt = 0.001)`
  - `sigma_0 < 2000` なら通常版、超過時は `sigma_0 / threshold + 1` 個のサブステップ版を使います。
  - サブステップ数は最大 20 に制限されます。
