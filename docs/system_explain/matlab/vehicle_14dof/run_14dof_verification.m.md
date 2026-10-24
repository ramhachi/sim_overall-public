# run_14dof_verification.m

## 目的
`run_14dof_verification.m` は、MATLAB 版 14DOF モデルを実行し、可能であれば C++ の結果 CSV と比較して合否判定まで行う検証スクリプトです。

## 入口
スクリプトです。`clear; clc; close all;` のあと、パラメータ設定、MATLAB 実行、単独結果可視化、C++ 結果探索と比較を順に進めます。

## 実行フェーズ
### 1. パラメータ設定
`params` を手作業で構築します。
- 質量・慣性: `m_sprung=200`、`track_r=1.18`、`Ixx=32`、`Iyy=95`、`Izz=85`、`Iw=0.15` など
- サスペンション: `k_spring=[25000,25000,30000,30000]`、`motion_ratio=[0.9,0.9,0.9,0.9]`
- Bouc-Wen、LuGre、ARB、バンプストップ
- タイヤ: `radius=0.228`、`mu_x_max=1.6`、`mu_y_max=2.0`、`Cx=80000`、`Cy=60000`、`v_N=0.5`、`k_tire=200000`、`c_tire=500`
- 空力: `rho=1.2`、`CL_A=2.5`、`CD_A=1.0`
- パワートレイン: `max_torque=55`、`gear_ratio=3.0`、`efficiency=0.9`
- 初速: `v0=15`

### 2. シミュレーション条件
- `t_span = [0, 5]`
- `dt = 0.001`
- ステア開始 `t_steer = 1.0`
- 舵角 `delta = 0.1`
- `input_func` は `throttle=0.1` と `steering=(t>=1.0)*0.1` を返します。

### 3. MATLAB シミュレーション実行
- `vehicle_14dof_model(params, t_span, dt, input_func)` を実行
- 実行時間を表示

### 4. 静的荷重と初期荷重の確認
- `W = m_sprung * 9.81`
- 前後軸荷重 `Wf`、`Wr` を重心位置で配分
- 静的荷重 `Wf/2`、`Wr/2` を表示
- さらに `data_m.Fz(1,:)` を「Initial Fz」として表示

### 5. MATLAB 単独結果の描画
2 行 4 列の図を作成します。
1. 軌跡
2. 前後速度 `vx`
3. ロール角
4. ピッチ角
5. タイヤ鉛直荷重
6. Bouc-Wen 力
7. LuGre 力
8. ヨーレート

### 6. 結果サマリー表示
- 最終位置
- 最終ヘディング角
- 最終速度
- 最大ロール角
- 最大ピッチ角
- `final_yaw_rate > 0.01` のときの旋回半径

### 7. C++ 結果探索
以下の 3 パスを順に探索します。
- `../output/14dof_result.csv`
- `../../output/14dof_result.csv`
- `14dof_result.csv`

最初に見つかったパスだけを使います。

### 8. C++ 比較処理
C++ CSV が見つかった場合のみ実行します。
- `readtable` で読み込み
- `t_cpp` を `dt` でインデックスに変換し、`idx_m = round(t_cpp / dt) + 1` として MATLAB 側の対応行を選びます
- 上限超過は `min(idx_m, size(state_m,1))` で切り詰めます

比較図は 2 行 3 列です。
1. 軌跡比較
2. ロール角比較
3. 前後速度比較
4. C++ CSV に `Fz_FL` 列がある場合のみ、左前輪鉛直荷重比較
5. ロール角誤差と RMSE
6. 位置誤差と最大値

### 9. 合否判定
- `max_err_phi < 1.0` 度
- `max_err_pos < 1.0` m

の両方を満たしたときだけ `PASS` を表示し、それ以外は `FAIL` です。

### 10. 画像保存
比較図が作られた場合のみ
- `../../output/verify_14dof.png`
に保存します。

## 外部依存
- `vehicle_14dof_model`
- `readtable`
- `saveas`
- 描画系関数

## このスクリプト内のローカル関数
ローカル関数はありません。

## しきい値と仮定
- ステア入力は 1 秒後に 0.1 rad の段差です。
- 旋回半径の表示条件は `final_yaw_rate > 0.01`
- 合否しきい値はロール誤差 1 度、位置誤差 1 m
- C++ 比較は CSV が見つかったときだけ実施します。

## 保存物
- C++ 比較図 `../../output/verify_14dof.png`（比較を実行した場合のみ）
