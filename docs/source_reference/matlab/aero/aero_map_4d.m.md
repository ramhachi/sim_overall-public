# `matlab/aero/aero_map_4d.m`

## 概要

`aero_map_4d` は、フロント／リアのライドハイト、ロール角、ヨー角を入力として空力係数を返す 4 次元空力マップクラスである。  
コンストラクタで既定グリッドとサンプル係数データを初期化し、`get_coeffs()` で係数取得、`compute_forces()` で空力・モーメント・前後ダウンフォース配分まで計算する。

## 主要関数・クラス

- `classdef aero_map_4d < handle`  
  4D 空力マップを保持するクラス。空気密度、参照面積、参照長、重心位置、既定係数、各グリッド、係数データ配列を持つ。
- `aero_map_4d()`  
  コンストラクタ。`init_default_grid()` と `init_sample_data()` を自動実行する。
- `init_default_grid()`  
  グリッドを定義する。  
  - RH_front: 20, 30, 40, 50, 60 mm  
  - RH_rear: 30, 40, 50, 60, 70, 80 mm  
  - roll: ±5° 範囲  
  - yaw: ±15° 範囲
- `init_sample_data()`  
  地面効果、ロール損失、ヨー感度を反映したサンプル係数場を生成する。
- `get_coeffs(rh_f_mm, rh_r_mm, roll_rad, yaw_rad)`  
  4D 線形補間で `CL`、`CD`、`CY`、`CoP_x`、`CoP_dist` を返す。
- `compute_forces(velocity, rh_f_mm, rh_r_mm, roll_rad, yaw_rad)`  
  動圧から `Fx`、`Fy`、`Fz`、`My`、`Mz`、`Fz_front`、`Fz_rear` を計算する。
- `interp4d(data, rh_f, rh_r, roll, yaw)`  
  `interpn()` を使った 4 次元線形補間。入力はグリッド範囲内へクランプされる。

## 理論・背景

このファイルは CFD の完全代替ではなく、設計初期や制御検証向けの準実用的な簡易空力モデルである。  
前後ライドハイト低下によりダウンフォースが増える地面効果、ロールによるフロアシール喪失、ヨーによるダウンフォース低下・ドラッグ増加・サイドフォース発生を係数式として組み込む。

また、圧力中心の前後移動を `CoP_x` と `CoP_dist` で表現しているため、単に総ダウンフォースを出すだけでなく、前後輪荷重配分まで一貫して評価できる。  
これにより 6/8/14DOF モデルでピッチ・ロール・タイヤ荷重の変化と空力の結合を扱いやすくしている。

## 外部ライブラリ・ツールボックス

外部ツールボックスは不要。  
使用する主な標準関数は `deg2rad()`、`interpn()`、`max()`、`min()` で、MATLAB 標準機能のみで完結する。

## 入出力

- 入力
  - `velocity` [m/s]
  - `rh_f_mm`, `rh_r_mm` [mm]
  - `roll_rad`, `yaw_rad` [rad]
- 出力
  - 係数: `CL`, `CD`, `CY`, `CoP_x`, `CoP_dist`
  - 力: `Fx`, `Fy`, `Fz`
  - モーメント: `My`, `Mz`
  - 前後配分: `Fz_front`, `Fz_rear`

## システム上の役割

MATLAB 側の空力基準モデルとして機能し、C++ 実装や上位車両モデルへ受け渡す空力応答の基盤を提供する。  
特に 6DOF、8DOF、14DOF で必要な「姿勢・車高依存の空力荷重」を軽量に再現する中核モジュールである。
