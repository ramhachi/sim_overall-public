# run_14dof_nn_simulation.m

## 目的
`run_14dof_nn_simulation.m` は、学習済み NN タイヤモデルを使った 14DOF シミュレーションを実行し、要約結果の表示、CSV 保存まで行うスクリプトです。コメントでは C++ 版と同条件の実行と、CSV 保存による比較可能化が目的として書かれています。

## 入口
スクリプトです。トップレベルを順に実行し、最後にローカル関数 `csvwrite_with_header` を定義しています。

## 実行フェーズ
### 1. 初期化と見出し表示
- `clear; clc;`
- タイトル文字列を `fprintf` で表示

### 2. 学習済みモデルの読み込み
- `model_path = '../dl_tire_model/output/ultra_deep_validation_model.mat'`
- `isfile(model_path)` で存在確認
- 無ければ `error(...)` で停止し、「先に学習せよ」という内容を出します。
- あれば `load(model_path)` を使い、
  - `trained_net` → `nn_model`
  - `norm_params` → `norm_params`
  を取り出します。

### 3. 車両パラメータ設定
`params` 構造体を手作業で作ります。

#### 質量
- `m_sprung = 200.0`
- `m_unsprung = [10.0, 10.0, 10.0, 10.0]`
- `a = 0.813`
- `b = 0.787`
- `L = params.mass.a + params.mass.b`
- `track_f = 1.2`
- `track_r = 1.2`
- `h_cg = 0.3`

#### 慣性
- `Ixx = 50.0`
- `Iyy = 100.0`
- `Izz = 80.0`
- `Iw = 0.5`

#### タイヤ
- `radius = 0.2286`
- `k_tire = 150000`
- `c_tire = 100`

#### サスペンション
- `k_spring = [30000, 30000, 30000, 30000]`
- `motion_ratio = [1.0, 1.0, 1.0, 1.0]`
- Bouc-Wen、LuGre、ARB、バンプストップの各定数を設定

#### パワートレイン
- `max_torque = 200.0`
- `gear_ratio = 10.0`
- `efficiency = 0.9`

#### 空力
- `rho = 1.225`
- `CL_A = 2.0`
- `CD_A = 0.8`

#### 環境
- `v0 = 15.0`
- `environment.T_ambient = 25.0`

### 4. ドライバ入力設定
`input_func` は次を返します。
- `steering = (t >= 1.0) * 0.1`
- `throttle = 0.1`
- `brake = 0.0`

### 5. シミュレーション実行
- `t_span = [0, 5.0]`
- `dt = 0.001`
- `vehicle_14dof_nn(params, t_span, dt, input_func, nn_model, norm_params)` を実行
- `tic` / `toc` で実行時間を計測

### 6. 結果抽出と表示
`state` から以下を分解します。
- 位置: `x`, `y`, `z`
- 姿勢: `phi`, `theta`, `psi`
- 速度: `vx`, `vy`, `vz`
- 角速度: `p`, `q`, `r`

さらに `speed = sqrt(vx.^2 + vy.^2)` を計算し、以下を表示します。
- 最終位置
- 最終方位角 `psi(end)` を度変換した値
- 最終速度
- 最大ロール角
- `abs(r(end)) > 0.01` のときの旋回半径 `speed(end) / abs(r(end))`
- 最終タイヤ鉛直荷重 `data.Fz(end,:)`
- 最終ダンパー力 `data.F_bw(end,:)` と `data.F_lugre(end,:)`
- 最終タイヤ表面温度 `data.T_surf(end,:)`

### 7. CSV 保存
- 出力ファイルは `14dof_nn_result.csv`
- 保存列は以下です。
  - `t`
  - `x`
  - `y`
  - `psi` の度変換
  - `vx`
  - `vy`
  - `phi` の度変換
  - `theta` の度変換
  - `data.ax`
  - `data.ay`
  - `data.Fz` の 4 列
- ヘッダーは
  `time,x,y,psi,vx,vy,phi,theta,ax,ay,Fz_FL,Fz_FR,Fz_RL,Fz_RR`
- いったん `fopen` / `fprintf` / `fclose` でヘッダーだけ書いたあと、`csvwrite_with_header` を呼びます。

## 外部依存
- `vehicle_14dof_nn`
- MATLAB `load`
- `isfile`
- `dlmwrite`

## ローカル関数 `csvwrite_with_header`
```matlab
function csvwrite_with_header(filename, data, header)
```

### 実際の動作
`dlmwrite(filename, data, '-append', 'precision', '%.6f');` だけを実行します。引数 `header` は受け取りますが使いません。ヘッダー行はメインスクリプト側で先に書かれているため、この関数は実質「数値データ追記専用」です。

## 仮定としきい値
- モデルファイルが無い場合は即時エラー終了です。
- 旋回半径は `abs(r(end)) > 0.01` を条件に表示します。
- ステア入力は 1 秒で段差入力です。

## 保存物
- `14dof_nn_result.csv`

## 描画
このスクリプトでは図を作成しません。表示はすべて `fprintf` のみです。
