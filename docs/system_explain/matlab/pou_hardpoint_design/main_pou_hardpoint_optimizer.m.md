# `matlab/pou_hardpoint_design/main_pou_hardpoint_optimizer.m`

## 1. 目的

POU システムの逆運動学最適化本体です。車両 JSON から目標多項式係数を取得し、M ピボットとベルクランク位置を `fmincon` で最適化して、幾何学曲線を目標ジャッキアップ曲線へ近づけます。

## 2. 入出力と依存関係

- 入力:
  - `params_json_file`: 最適化済み車両 JSON へのパス
  - `options`:
    - `wheel_index` (既定 0)
    - `n_multistart` (既定 5)
    - `delta_range_deg` (既定 `[-30, 30]`)
    - `n_delta_points` (既定 61)
    - `verbose` (既定 `true`)
    - `override_x`（存在時は最適化を飛ばして設計変数を強制使用）
- 出力 `results`:
  - `M_pivot_upright`, `M_pivot_chassis`, `bellcrank_pivot`
  - `polynomial_error_mm`, `max_jack_error_mm`, `max_camber_error_deg`
  - `max_rod_angle_deg`, `installation_ratio`, `packaging_valid`, `violations`
  - `optimization_exitflag`, `optimization_time_s`, `pushrod_length_nominal_m`
  - 描画用の `delta_range_deg`, `jack_*`, `camber_*`
- 依存関係:
  - `load_vehicle_artifact`
  - `define_suspension_geometry`
  - `objective_polynomial_fit`
  - `constraint_rod_misalignment`
  - `generate_initial_guess`
  - `solve_upright_position_3d`
  - `forward_kinematics`
  - `validate_packaging`
  - MATLAB Optimization Toolbox の `fmincon`, `optimoptions`

## 3. 全体処理フロー

1. `options` の不足フィールドを既定値で補完します。
2. `load_vehicle_artifact` を使えるよう、必要なら親ディレクトリを `addpath` します。
3. JSON を読み込み、`params.suspension` からホイール別 POU 多項式係数を取得します。
4. POU 係数が無い場合は `jack_c1=1.0`, `jack_c2=0.5`, `camber_c1=0`, `camber_c2=0` を代替値として使い、`warning` を出します。
5. `define_suspension_geometry` でサスペンション形状、境界、初期値、派生情報を得ます。
6. `delta_range` を操舵角配列として作り、設計変数 `[Xm, Ym, Zm, Xb, Yb, Zb]` の上下限を組みます。
7. 境界が壊れていないか `lb >= ub` で検査し、破綻していれば即 `error` で停止します。
8. 目的関数 `obj_fun` と制約関数 `con_fun` を関数ハンドルで作ります。
9. `fmincon` を SQP で設定し、中央差分、`ConstraintTolerance=2e-3`、`ScaleProblem=true` などを与えます。
10. `override_x` があればその値を採用し、最適化をスキップします。
11. それ以外では `aggressive / nominal / conservative / random / random` の戦略で初期値を作り、複数回 `fmincon` を走らせ、最も良い解を保持します。
12. 収束解が無ければ最後に得た `x_opt` を暫定採用し、それすら無ければ失敗します。
13. 最良解から 0 操舵時の chassis 座標、ジャッキアップ曲線、キャンバー曲線、誤差、ロッド角、IR、パッケージング判定を計算します。
14. `results` を構築し、`verbose` 時は要約を表示します。

## 4. 関数詳細

### `main_pou_hardpoint_optimizer(params_json_file, options)`

- **目標係数の取り出し**  
  `idx = wheel_index + 1` で MATLAB の 1 始まりへ変換し、`susp.pou_jack_c1/c2`, `susp.pou_camber_c1/c2` を読みに行きます。

- **境界の構築**  
  `geom.M_pivot_bounds` と `geom.bellcrank_bounds` を縦結合して 6 変数分の上下限を作ります。

- **マルチスタートの採点基準**  
  `exitflag >= 1` かつ最小コストを最優先に採用します。`exitflag == 0` でも、他候補が無いか、既存ベストの 1.5 倍以内なら部分解として受け入れます。

- **座標変換**  
  `best_x(1:3)` は upright 座標系の M ピボットです。0 操舵の `solve_upright_position_3d` を通して chassis 座標へ変換したものを `M_pivot_chassis` として保存します。

- **誤差計算**  
  ジャッキアップ誤差は mm、キャンバー誤差は deg に変換して要約値を出します。`polynomial_error_mm` はジャッキアップ RMS のみです。

- **ロッド角の扱い**  
  `constraint_rod_misalignment` の 1 番目の不等式値 `c(1)` が「最大ロッド角 - 許容値」なので、`c(1) + geom.rod_end_angle_max` から実角度を復元しています。

- **インストレーション比**  
  0 操舵の M ピボットからキングピン軸への垂直距離を、0 操舵 pushrod 長で割って求めます。

- **ローカル関数**  
  このファイルにローカル関数はありません。
