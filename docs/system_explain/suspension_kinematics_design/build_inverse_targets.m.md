# `suspension_kinematics_design/build_inverse_targets.m`

## 役割

車両 artifact の runtime パラメータから、front / rear inverse-analysis の目標 struct を作る helper です。従来の固定 target 群ではなく、`params.suspension` と `params.suspension.kc.*` を読み、現行 runtime 契約に近い目標群を返します。

## 処理の流れ

1. `axleKey` に応じて front / rear の既定値と対象 wheel index を決める。
2. static camber / toe、RC、KPI、caster、camber gain などの初期値を設定。
3. `params.suspension.kc.front|rear` があれば axle-level target を上書き。
4. `params.suspension.kc.corners.<wheel>` があれば corner-level kinematics / compliance を上書き。
5. 最後に目的関数用 `weights` を付与して返す。

## 返す主な target

- static camber / toe
- roll center
- KPI / caster / scrub / mechanical trail
- `camber_vs_travel`, `toe_vs_travel`
- `camber_vs_steer`, `toe_vs_steer`
- roll gain / RC migration / jacking
- anti geometry
- compliance gradients

## システム上の位置づけ

最適化本体を runtime artifact へ従属させる target builder です。
