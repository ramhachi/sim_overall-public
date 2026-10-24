# `matlab/rl_driver/ensure_rl_driver_paths.m`

## 役割

RL ドライバー系関数を MATLAB から直接呼んだときの path 不足を補う bootstrap です。`matlab/rl_driver/` と親 `matlab/` を path へ追加します。

## 処理の流れ

1. `mfilename('fullpath')` から `rl_driver` ディレクトリを取る。
2. `addpath_if_missing(thisDir)`。
3. `addpath_if_missing(fileparts(thisDir))`。

## 補助関数

### `addpath_if_missing(dirPath)`
- 空・非存在ディレクトリは無視
- 現在の `path` を `pathsep` で分解
- 完全一致が無いときだけ `addpath`

## システム上の位置づけ

EasyStart を介さず resume / benchmark / export helper を叩く経路の安全装置です。
