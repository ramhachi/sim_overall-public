# `matlab/rl_driver/ensure_rl_driver_paths.m`

## 概要

`matlab/rl_driver/` とその親 `matlab/` を MATLAB path へ追加する最小 helper です。RL 関数を直接叩いたときの path 不足を防ぎます。

## 主な入力 / 出力

- 入力なし
- 戻り値なし

## システム上の役割

resume / benchmark / export などの RL 入口を、EasyStart 以外から実行しても壊れにくくする bootstrap です。
