# `suspension_kinematics_design/build_inverse_targets.m`

## 概要

車両 artifact の runtime パラメータから、front / rear の逆解析目標を組み立てる helper です。静的値だけでなく、`params.suspension.kc.*` に既に入っている K&C / compliance surrogate も読み込みます。

## 主な入力

- `params`
- `axleKey` (`front` / `rear`)

## 主な出力

- `targets`
  - static camber / toe
  - roll center / KPI / caster
  - camber / toe vs travel / steer
  - anti / jacking
  - compliance gradients
  - optimization weights

## システム上の役割

サスペンション最適化を「コード内固定目標」から「runtime artifact 駆動目標」へ変える橋渡しです。
