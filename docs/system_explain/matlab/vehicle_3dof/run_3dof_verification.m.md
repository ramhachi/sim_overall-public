# `matlab/vehicle_3dof/run_3dof_verification.m`

## 役割

このスクリプトは、`vehicle_3dof_model` で計算した MATLAB 側 3 自由度車両モデルの結果を、C++ 実装が出力した CSV と比較する検証用スクリプトです。  
処理対象は平面運動の `x`, `y`, `psi`, `vx`, `vy`, `r` で、誤差の集計、可視化、MATLAB 結果の CSV 保存までを 1 本で実行します。

## 実行入口

- スクリプトファイルです。引数は取りません。
- 冒頭で `clear; clc; close all;` を実行し、ワークスペース・コンソール・図を初期化します。

## 外部依存

- `vehicle_3dof_model(params, t_span, dt, input_func)`
- `readtable`
- `interp1`
- `rad2deg`
- `saveas`
- `writetable`
- C++ 側の結果ファイル
  - 第一候補: `build/3dof_step_steer_result.csv`
  - 第二候補: `output/3dof_step_steer_result.csv`

どちらの CSV も存在しない場合は、`error('C++ result file not found ... Run ./cpp/fsae_3dof first.')` で停止します。

## パラメータ設定

コード内で直接 `params` 構造体を組み立てます。主な設定値は次の通りです。

- 質量系
  - `m_sprung = 240`
  - `m_unsprung = [10, 10, 10, 10]`
  - `L = 1.55`, `a = 0.73`, `b = 0.82`
  - `track_f = 1.2`, `track_r = 1.15`
  - `Izz = 85`, `Iw = 0.15`
- タイヤ
  - `radius = 0.228`
  - `mu_x_max = 1.6`, `mu_y_max = 1.8`
  - `Lx = 0.15`, `Ly = 0.25`
  - `Cx = 80000`, `Cy = 60000`
  - `v_N = 0.5`
- 空力
  - `rho = 1.225`
  - `CD_A = 0.8`
- 駆動系
  - `max_torque = 55`
  - `gear_ratio = 3.0`
  - `efficiency = 0.9`
- 初期速度
  - `v0 = 15.0`

## 入力条件

入力は無名関数 `input_func` で与えます。

- `step_time = 1.0`
- `step_steer = 0.1` rad
- `throttle = 0.3`

`t >= 1.0` のときだけ操舵角を `0.1` rad にし、スロットルは全時刻で一定です。

## 処理の流れ

### 1. MATLAB シミュレーション

- `t_span = [0, 5]`
- `dt = 0.001`

で `vehicle_3dof_model` を呼び、`tic` / `toc` で実行時間を表示します。  
戻り値 `data_matlab` は受け取りますが、このスクリプト内では後続処理に使っていません。

### 2. C++ 結果の読込み

`readtable` で C++ 結果 CSV を読み込み、件数を表示します。

### 3. MATLAB 結果の比較形式への変換

状態配列 `state_matlab` から次を抽出します。

- `x = state_matlab(:,1)`
- `y = state_matlab(:,2)`
- `psi = rad2deg(state_matlab(:,3))`
- `vx = state_matlab(:,4)`
- `vy = state_matlab(:,5)`
- `r = rad2deg(state_matlab(:,6))`

角度系だけ度へ変換しており、C++ CSV 側の単位に合わせています。

### 4. 時間軸合わせ

C++ 側の `time` を基準に、`interp1(..., 'linear', 'extrap')` で MATLAB 結果を線形補間します。  
補間対象は `x`, `y`, `psi`, `vx`, `vy`, `r` の 6 変数です。

### 5. 誤差計算

比較項目と単位は次の固定配列で定義されています。

- 変数: `{'x', 'y', 'psi', 'vx', 'vy', 'r'}`
- 単位: `{'m', 'm', 'deg', 'm/s', 'm/s', 'deg/s'}`
- 許容誤差: `[1.0, 1.0, 5.0, 0.5, 0.5, 5.0]`

各変数について以下を計算します。

- 最大絶対誤差 `max_abs`
- RMSE `rmse`
- 平均誤差 `mean_err`
- 時系列誤差 `diff`

`max_abs < tolerance` のときだけその変数を合格とし、1 つでも超過すると全体結果 `all_pass` は偽になります。

### 6. プロット

`figure('Position', [100, 100, 1400, 900])` で 2 行 3 列の図を作成します。

1. 軌跡比較
2. ヨー角比較
3. 縦速度比較
4. 横速度比較
5. ヨーレート比較
6. 速度誤差
   - `vx` 誤差
   - `vy` 誤差
   - `r` 誤差を `10` で割ったもの

最後に `sgtitle('3DOF Vehicle Model: C++ vs MATLAB Verification')` を付けます。

### 7. 画像保存

`output` ディレクトリがなければ `mkdir` で作成し、図を `output/verify_3dof.png` に保存します。

### 8. MATLAB 結果の CSV 出力

MATLAB 計算結果だけを `output/3dof_matlab_result.csv` に保存します。  
列名は `time`, `x`, `y`, `psi`, `vx`, `vy`, `r` です。

## 補足事項

- 検証閾値はコード内に固定値で埋め込まれており、外部設定化はされていません。
- C++ 結果探索は `build` を先に、`output` を後に見る 2 段階です。
- 角度比較は MATLAB 側のみ `rad2deg` しており、C++ 側 CSV は度前提です。
- ローカル関数はありません。処理は上から順に実行される単純な検証スクリプトです。
