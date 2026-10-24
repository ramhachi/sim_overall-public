# vehicle_params.hpp

## 1. ファイルの目的
車両全体の設定値を階層構造で保持し、JSON から読み込むためのヘッダです。14DOF 本体、簡略モデル、各ドライバーが共通利用するパラメータ定義の中心です。

## 2. 実行上の役割
全車両モデルの初期化時に使われる「設定の入口」です。ネスト JSON と Python/RL 側のフラット JSON の両方を受けられるよう `parse_json()` が作られています。

## 3. 依存関係
- `vehicle_14dof_new.hpp`、`vehicle_3dof.hpp`、`vehicle_6dof.hpp`、`vehicle_8dof*`、`bicycle_model*`
- JSON 読み込み経路全般

## 4. 入力/出力
- 入力: `nlohmann::json`、JSON ファイルパス、JSON 文字列
- 出力: `VehicleParams`

## 5. 関数・メソッド・クラスの網羅的な説明
### `enum WheelIndex`
`FL`, `FR`, `RL`, `RR` の車輪インデックスです。

### `struct MassParams`
質量・重心・慣性の設定です。`m_total`, `m_sprung`, `m_unsprung`, `wheelbase`, `a`, `b`, `track_f`, `track_r`, `h_cg`, `Ixx`, `Iyy`, `Izz`, `Ixz`, `Iw` を持ちます。

### `struct SuspensionParams`
サスペンション設定です。
- 基本値: `k_spring`, `motion_ratio`, `k_arb_f`, `k_arb_r`
- Bouc-Wen: `c0`, `k0`, `bw_alpha`, `bw_A`, `bw_beta`, `bw_gamma`, `bw_n`
- LuGre: `lugre_sigma0/1/2`, `lugre_Fc`, `lugre_Fs`, `lugre_vs`, `lugre_substeps`
- その他: `bump_stop_gap`, `bump_stop_k`, `camber_gain`, `static_camber`, `static_toe`, `pou_jack_c1/c2`, `pou_camber_c1/c2`, `K_chassis`, `C_chassis`, `I_twist`

#### `struct SuspensionParams::POUHardpoint`
POU ハードポイント参照値。`M_pivot_chassis`, `M_pivot_upright`, `bellcrank_pivot`, `pushrod_length`, `installation_ratio`, `valid` を持ちます。

#### `struct SuspensionParams::KCCornerParams`
コーナーごとの K&C サロゲートです。
- `Hardpoints`: アーム、タイロッド、プッシュロッド、ダンパー、ARB リンク等の 3D 座標
- `KinematicSurrogate`: `camber_vs_travel`, `toe_vs_travel`, `camber_vs_steer`, `toe_vs_steer`, `roll_camber_gain`, `roll_steer_gain`, `jacking_force_per_fy`
- `ComplianceSurrogate`: `toe/camber/caster/track/wheelbase_per_load`
- 幾何量: `caster`, `kpi`, `sai`, `scrub_radius`, `mechanical_trail`, `caster_trail`, `valid`

#### `struct SuspensionParams::KCAxleParams`
アクスル単位の K&C 値です。`roll_center_height`, `roll_center_lateral`, `roll_center_height_per_travel`, `roll_center_height_per_roll`, `anti_brake_pct`, `anti_drive_pct`, `anti_lift_pct`, `anti_rise_pct`, `arb_motion_ratio` を持ちます。

### `struct TireParams`
タイヤ設定です。`radius`, `mu_x_max`, `mu_y_max`, `Fz_nom`, `Lx`, `Ly`, `Cx`, `Cy`, `v_N`, `k_tire`, `c_tire`, `dmu_dFz`, `pneumatic_trail`, `model_type`, `model_dir`, `model_manifest`, `model_name`, `ambient_temperature_c` を持ちます。

#### `struct TireParams::RelaxationParams`
荷重・温度依存緩和長設定です。`Lx_base`, `Ly_base`, `Fz_exponent`, `temp_coeff_per_c`, `temp_ref_c`, `min/max_Lx`, `min/max_Ly`, `enabled` を持ちます。

### `struct SteeringParams`
操舵系設定です。`rack_ratio`, `ackermann`, `max_angle`, `stiffness`, `I_steer`, `R_steer`, `T_fric`, `C_steer_torque`, `K_return`, `T_driver_max`, `torque_mode` を持ちます。

### `struct PowertrainParams`
エンジン・ギア・LSD 設定です。`engine_rpm_points`, `engine_torque_curve`, `gear_ratios`, `primary_ratio`, `final_ratio`, `efficiency`, 後方互換用 `max_torque`, `gear_ratio`, および LSD 用 `lsd_*` 群を持ちます。

### `struct BrakeParams`
`max_pressure`, `balance`, `mu_pad`, `caliper_area`, `effective_radius` を持つブレーキ設定です。

### `struct AeroParams`
`CL_A`, `CD_A`, `aero_balance`, `h_aero`, `rho`, `map_mode`, `aero_map_file`, 各種基準条件、`Fx/Fy/Fz/Mx/My/Mz_ref_total`, `planner_uses_aero_map` を持つ空力設定です。

### `struct VehicleParams`
上記すべてを統合したルート構造体です。
- `parse_json(...)`: ネスト形式とフラット形式の両方に対応し、mass/suspension/tire/steering/powertrain/aero/brake、POU ハードポイント、K&C 情報まで読み込む中心関数
- `from_json(const std::string&)`: ファイルから読み込む
- `from_json_string(const std::string&)`: 文字列から読み込む
- `default_fsae()`: デフォルト値を返す
