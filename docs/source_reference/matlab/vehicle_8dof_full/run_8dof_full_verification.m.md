# `matlab/vehicle_8dof_full/run_8dof_full_verification.m`

## 概要

`run_8dof_full_verification.m` は、Bouc-Wen + LuGre を統合した 8DOF Full モデルの検証スクリプトである。  
C++ 比較ファイルがある場合はロール角、ダンパ力、摩擦力、摩擦内部状態を比較し、なければ MATLAB 単体で各量を可視化する。

## 主要関数・クラス

- `vehicle_8dof_full_model(...)`  
  MATLAB 側シミュレーション本体。
- C++ 結果探索部  
  `lugre_comparison.csv` を複数パスから探索する。
- 比較項目  
  - `phi_full`
  - `damper_force_fl`
  - `friction_force_fl`
  - `friction_z_fl`
- 可視化部  
  6 パネルでロール角、誤差、Bouc-Wen 力、LuGre 力、LuGre 状態、総サスペンション力を表示する。

## 理論・背景

このスクリプトの狙いは、姿勢応答が合うかだけでなく、摩擦モデル内部まで C++ と整合するかを確認することにある。  
LuGre は低速域の stick-slip や Stribeck 効果を表すため、摩擦力だけでなく内部状態 `friction_z` の比較が重要になる。

また、Bouc-Wen 力と LuGre 力を別々に見たうえで合計力を表示するため、減衰源ごとの寄与分離がしやすい。  
結果として、14DOF に進んだ際のサスペンション起因の差異解析にも役立つ。

## 外部ライブラリ・ツールボックス

外部ツールボックスは不要。  
標準の入出力・補間・描画機能のみを利用する。

## 入出力

- 入力
  - 代表 LuGre パラメータ  
    - `sigma0=10000`
    - `Fc=20`
    - `Fs=30`
    - `vs=0.01`
  - C++ 結果: `lugre_comparison.csv`
- 出力
  - 比較図 `../../output/verify_8dof_full.png`
  - コンソール上のロール誤差、摩擦関連比較情報

## システム上の役割

完全サスペンションモデルの検証ハーネスとして、Bouc-Wen と LuGre の同時統合結果を評価する。  
14DOF 完全車両へ上げる前の最終サスペンション検証段階であり、詳細モデルの信頼性を確保する役目を持つ。
