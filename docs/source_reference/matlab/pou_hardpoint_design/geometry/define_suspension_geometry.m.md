# `matlab/pou_hardpoint_design/geometry/define_suspension_geometry.m`

## 概要

POU 設計で使う `geom` 構造体を組み立てる関数です。車両寸法を読みつつ、前輪は vehicle artifact の `params.suspension.kc.corners.*.hardpoints` から A-arm / tie-rod 幾何を取り込みます。

## 主な入力 / 出力

- 入力: `wheel_index`, 任意の `vehicle_params`
- 出力: `geom`

## 重要な挙動

- front は artifact hardpoints を優先し、FR は FL を Y 反転して作れます。
- tie-rod が無い場合だけ front-steer の既定値へフォールバックします。
- upper/lower arm axis、kingpin axis、bellcrank / M-pivot bounds までまとめて返します。

## システム上の役割

artifact-first A-arm hardpoints と POU 最適化・順運動学を接続する前処理です。
