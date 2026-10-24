# run_14dof_cpp_comparison.m

## 目的
`run_14dof_cpp_comparison.m` は、TMeasy タイヤ版の MATLAB 14DOF 車両モデルを実行し、CSV 出力と任意の C++ 結果比較まで行う検証スクリプトです。冒頭コメントでは、NN モデル版ではなく TMeasy 版を使う理由として「C++ 側で問題あり調査中」と明記されています。

## 入口
このファイルはスクリプトです。実行するとトップレベル文が順に走り、最後にローカル関数 `setup_params_cpp_compatible` が使われます。

## 実行フェーズ
### 1. 初期化
- `clear; clc; close all;`
- 罫線付きタイトルを表示

### 2. パラメータ設定
- `setup_params_cpp_compatible()` を呼び、`params` を生成します。
- ここで質量、サスペンション、タイヤ、空力、パワートレイン、初速をまとめて設定します。

### 3. シミュレーション条件設定
- `t_span = [0, 5]`
- `dt = 0.001`
- `t_steer = 1.0`
- `delta = 0.1` `rad`
- `input_func` は以下を返します。
  - `throttle = 0.3`
  - `steering = (t >= t_steer) * delta`
  - `gear = 3`

### 4. MATLAB シミュレーション実行
- `vehicle_14dof_model(params, t_span, dt, input_func)` を呼びます。
- 実行時間を `tic` / `toc` で計測します。
- 結果は `t_m`、`state_m`、`data_m` に格納します。

### 5. 結果分析
最終時刻の以下を表示します。
- 位置 `x_final`、`y_final`
- ヨー角 `psi_final` を度へ変換した値
- 速度 `vx_final`、`vy_final`
- ヨーレート `r_final` を度毎秒へ変換した値
- タイヤ平均鉛直荷重 `mean(data_m.Fz, 1)`

### 6. MATLAB 結果の描画
8 分割図を作ります。
1. 軌跡
2. `vx` / `vy`
3. ヨーレート
4. ロール角
5. タイヤ鉛直荷重
6. `ax` / `ay`
7. タイヤ横力
8. タイヤ前後力

### 7. CSV 出力
- 出力名: `matlab_14dof_result.csv`
- 事前に各列サイズを表示し、列ベクトル化した後で横結合します。
- ヘッダーは
  `time,x,y,psi,vx,vy,r,ax,ay,Fz_FL,Fz_FR,Fz_RL,Fz_RR`
- `fopen` / `fprintf` / `fclose` でヘッダーを書き、その後 `dlmwrite(..., '-append', 'precision', '%.6f')` で数値本体を追記します。

### 8. 任意の C++ 結果比較
- 対象ファイルは固定で `../../cpp/build/step_steer_result.csv`
- 存在する場合は `readtable` で読み、3 分割図を描きます。
  1. 軌跡比較
  2. 前後速度比較
  3. ヨーレート比較
- `interp1(..., 'linear', 'extrap')` で C++ 側データを MATLAB 時刻列 `t_m` に合わせます。
- 比較指標として以下を表示します。
  - `vx` RMS 誤差
  - `r` RMS 誤差
  - `vx` 最大誤差
  - `r` 最大誤差
- ファイルが無い場合はパスを表示し、C++ 実行コマンドの案内だけ出します。

## 外部依存
- `vehicle_14dof_model`
- `readtable`
- `interp1`
- `rms`
- 描画系 `figure`、`subplot`、`plot`、`legend` など

## ローカル関数 `setup_params_cpp_compatible`
```matlab
function params = setup_params_cpp_compatible()
```

### 役割
C++ 互換を意図したパラメータ構造体を返します。

### 設定内容
#### 質量・慣性
- `m_sprung = 200`
- `m_unsprung = [10, 10, 10, 10]`
- `a = 0.813`
- `b = 0.787`
- `L = params.mass.a + params.mass.b`
- `track_f = 1.2`
- `track_r = 1.18`
- `h_cg = 0.28`
- `h_roll = 0.28`
- `Ixx = 32`
- `Iyy = 95`
- `Izz = 85`
- `Iw = 0.15`

#### サスペンション
- `k_spring = [25000, 25000, 30000, 30000]`
- `motion_ratio = [0.9, 0.9, 0.9, 0.9]`
- Bouc-Wen: `bw_c0 = 2000` など
- LuGre: `lugre_sigma0 = 10000` など
- `k_arb_f = 5000`
- `k_arb_r = 3000`
- `bump_stop_gap = 0.03`
- `bump_stop_k = 100000`

#### タイヤ
- コメント上は Hoosier 実測データベース
- `radius = 0.228`
- `mu_x_max = 1.8`
- `mu_y_max = 2.0`
- `Fz_nom = 1500`
- `Lx = 0.15`
- `Ly = 0.25`
- `Cx = 80000`
- `Cy = 22000`
- `v_N = 0.5`
- `k_tire = 200000`
- `c_tire = 500`

#### 空力
4D マップ用として次を設定します。
- `rho = 1.225`
- `A_ref = 1.0`
- `L_ref = 1.6`
- `x_cg = 0.8`
- `CL_default = -2.5`
- `CD_default = 1.0`
- `CoP_default = 0.45`

#### パワートレイン
- `gear_ratio = 3.0`
- `efficiency = 0.95`

#### 初速
- `v0 = 20`

## しきい値・仮定
- ステア入力は `t >= 1.0` で急に 0.1 rad へ切り替わります。
- 比較対象 C++ CSV パスは固定で探索処理はありません。
- CSV は 13 列を期待してサイズ表示しています。

## 保存物
- `matlab_14dof_result.csv`
- 画面上の図は作成しますが、このファイルでは `saveas` はしていません。
