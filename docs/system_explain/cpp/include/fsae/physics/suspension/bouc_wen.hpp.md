# bouc_wen.hpp

## ファイルの目的
- サスペンションダンパのヒステリシスを表す Bouc-Wen モデルを提供します。

## 実行上の役割
- 変位・速度・履歴変数から減衰力と履歴変数微分を返し、外部積分器で状態更新できるようにしています。

## 依存関係
- 外部: `<cmath>`
- リポジトリ内依存: なし

## 入力/出力
- 入力: `x`, `v`, `z`, `BoucWenParams`
- 出力: 力 `force`、履歴変数微分 `dz_dt`

## 関数・メソッド・クラスの網羅的な説明
- `template<typename Scalar> struct BoucWenParams`
  - `c0`, `k0` は粘性減衰と線形ばね項です。
  - `alpha`, `A`, `beta`, `gamma`, `n` はヒステリシス形状を決めます。
  - `x0` は中立位置です。
- `compute_bouc_wen_force(const BoucWenParams<Scalar>& params, Scalar x, Scalar v, Scalar z)`
  - `c0*v + k0*(x-x0) + alpha*z` を返します。
- `compute_bouc_wen_dz(const BoucWenParams<Scalar>& params, Scalar v, Scalar z)`
  - `A*v - beta*|v|*z*|z|^(n-1) - gamma*v*|z|^n` を計算します。
- `template<typename Scalar> struct BoucWenOutput`
  - `force` と `dz_dt` をまとめた返却用構造体です。
- `compute_bouc_wen(const BoucWenParams<Scalar>& params, Scalar x, Scalar v, Scalar z)`
  - 上の 2 関数を同時に呼び、`BoucWenOutput` として返します。
