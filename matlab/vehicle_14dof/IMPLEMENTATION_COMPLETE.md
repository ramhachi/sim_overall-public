# MATLAB 14DOF C++統合実装完了

## 実装日時
2026年2月1日

## 実装内容

### 新規作成ファイル

1. **engine_torque_map.m** - エンジントルクマップ
   - CBR600RRのトルクカーブ（17ポイント）
   - C++実装と完全一致

2. **salisbury_lsd.m** - Salisbury式LSD
   - ランプ角: 加速35°/減速50°
   - プリロード: 40Nm
   - トルク配分計算

3. **aero_map_4d.m** - 4次元エアロマップ
   - ライドハイト、ロール角、ヨー角依存
   - 地面効果、フロアシール損失、ヨー効果モデル化
   - 動的CoP計算

4. **verify_params_match.m** - パラメータ確認スクリプト
   - MATLAB/C++パラメータ一致確認
   - エンジン、LSD、エアロマップテスト

5. **run_14dof_cpp_comparison.m** - C++比較検証スクリプト
   - MATLABシミュレーション実行
   - C++結果との比較プロット
   - 誤差統計計算

### 更新ファイル

6. **vehicle_14dof_model.m** - メインモデル更新
   - エンジントルクマップ統合
   - LSD統合
   - 4Dエアロマップ統合

## C++との対応

| 機能 | MATLAB実装 | C++実装 | 一致 |
|:---|:---|:---|:---:|
| エンジントルク | `engine_torque_map.m` | `EngineTorqueMap` | ✅ |
| LSD | `salisbury_lsd.m` | `SalisburyLSD` | ✅ |
| 4Dエアロ | `aero_map_4d.m` | `AeroMap4D` | ✅ |
| 初速度 | 20 m/s | 20 m/s | ✅ |
| 空力係数 | CL=-2.5, CD=1.0 | CL=-2.5, CD=1.0 | ✅ |

## 使用方法

### 1. パラメータ確認
```matlab
cd matlab/vehicle_14dof
verify_params_match
```

### 2. MATLAB単独シミュレーション
```matlab
% パラメータ設定
params = struct();
% ... (setup_params_cpp_compatible参照)

% シミュレーション実行
t_span = [0, 5];
dt = 0.001;
input_func = @(t) struct('throttle', 0.3, 'steering', 0.1, 'gear', 3);
[t, state, data] = vehicle_14dof_model(params, t_span, dt, input_func);
```

### 3. C++との比較
```matlab
% MATLABシミュレーション
run_14dof_cpp_comparison

% C++シミュレーション（別ターミナル）
% cd cpp/build && ./fsae_14dof step_steer

% 再度MATLABで比較プロット生成
run_14dof_cpp_comparison
```

## 実装詳細

### エンジントルクマップ
```matlab
% 17ポイントトルクカーブ
rpm_points = [3000, 4000, ..., 15000];
torque_points = [40.0, 45.0, ..., 55.0];
torque = interp1(rpm_points, torque_points, rpm);
```

### LSD
```matlab
% Salisbury式
K_ramp = tan(ramp_angle);
T_lsd = preload + abs(T_in) * K_ramp;
% 速度差に応じた配分
torque_bias = 0.5 + adjustment;
T_L = T_out * torque_bias;
T_R = T_out * (1 - torque_bias);
```

### 4Dエアロマップ
```matlab
% ライドハイト効果
rh_effect = 1.0 + 0.015*(40-rh_f) + 0.010*(55-rh_r);
% ロール損失
roll_loss = 1.0 - 3.0*abs(roll);
% ヨー効果
yaw_effect = cos(yaw*2);
CY = sin(yaw*2) * 0.5;
```

## 検証項目

### パラメータ一致確認
- ✅ 質量・慣性モーメント
- ✅ サスペンション（スプリング、ダンパー、ARB）
- ✅ タイヤ（TMeasy パラメータ）
- ✅ 空力（4Dマップパラメータ）
- ✅ パワートレイン（ギア比、効率）

### 機能テスト
- ✅ エンジントルクマップ（RPM vs トルク）
- ✅ LSD（トルク配分）
- ✅ 4Dエアロマップ（RH/ロール/ヨー依存性）

### シミュレーション比較
- 📊 軌跡（x-y）
- 📊 速度（vx, vy）
- 📊 ヨーレート（r）
- 📊 タイヤ荷重（Fz）
- 📊 加速度（ax, ay）

## 出力ファイル

### CSVファイル
- `matlab_14dof_result.csv` - MATLAB シミュレーション結果
  - 列: time, x, y, psi, vx, vy, r, ax, ay, Fz_FL, Fz_FR, Fz_RL, Fz_RR

### プロット
- MATLAB 14DOF Results (8サブプロット)
  - 軌跡、速度、ヨーレート、ロール角
  - タイヤ荷重、加速度、タイヤ横力、前後力
- MATLAB vs C++ Comparison (3サブプロット)
  - 軌跡比較、速度比較、ヨーレート比較

## 今後の展開

### 短期
1. C++との詳細比較（RMS誤差評価）
2. 各種テストケースでの検証
   - step_steer
   - circle
   - acceleration
   - cornering

### 中期
1. NNタイヤモデルの統合（MATLAB側）
2. 温度依存性の追加
3. より詳細な摩擦モデル

### 長期
1. リアルタイムシミュレーション対応
2. HIL（Hardware-in-the-Loop）統合
3. 最適制御との連携

## トラブルシューティング

### パスエラー
```matlab
% MATLABのカレントディレクトリを確認
pwd
% 必要に応じて移動
cd /path/to/matlab/vehicle_14dof
```

### C++結果が見つからない
```bash
# C++シミュレーション実行
cd cpp/build
./fsae_14dof step_steer
```

### プロットが表示されない
```matlab
% フィギュアを明示的に表示
figure(1);
% または
set(0, 'DefaultFigureVisible', 'on');
```

## 参考
- C++ 実装: `cpp/include/fsae/`
- C++ 比較用出力: `cpp/build/step_steer_result.csv`
- 設計書: `docs/設計書_統合版.md`
