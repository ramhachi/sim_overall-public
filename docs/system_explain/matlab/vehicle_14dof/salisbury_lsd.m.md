# salisbury_lsd.m

## 目的
`Salisbury` 式 LSD を模した簡易トルク配分関数です。左右車輪角速度差と入力トルクから、左右出力トルク `T_L`、`T_R` とデフ出力軸角速度 `omega_out` を返します。

## 入口とシグネチャ
```matlab
function [T_L, T_R, omega_out] = salisbury_lsd(omega_L, omega_R, T_in, params)
```

### 入力
- `omega_L`: 左車輪角速度 `[rad/s]`
- `omega_R`: 右車輪角速度 `[rad/s]`
- `T_in`: 入力トルク `[Nm]`
- `params`: LSD パラメータ
  - `ramp_accel` `[deg]`
  - `ramp_decel` `[deg]`
  - `preload` `[Nm]`
  - `efficiency` `[0-1]`

### 出力
- `T_L`: 左輪トルク `[Nm]`
- `T_R`: 右輪トルク `[Nm]`
- `omega_out`: 左右平均角速度 `[rad/s]`

## 外部依存
外部関数はありません。ローカル関数もありません。

## 既定値
`params` が無い、または各フィールドが無い場合は次を補完します。
- `ramp_accel = 35.0`
- `ramp_decel = 50.0`
- `preload = 40.0`
- `efficiency = 0.95`

## 処理の流れ
1. 加減速ランプ角を度からラジアンへ変換します。
2. `omega_out = 0.5 * (omega_L + omega_R)` を計算します。
3. `delta_omega = omega_R - omega_L` を計算します。
4. `T_in >= 0` なら加速側ランプ角、負なら減速側ランプ角を選びます。
5. `K_ramp = tan(ramp_angle)` を求めます。
6. LSD 制限トルクを
   ```matlab
   T_lsd = preload + abs(T_in) * K_ramp
   ```
   で作ります。
7. トルク配分比 `torque_bias` を決めます。
   - `abs(delta_omega) < 0.01` なら `0.5`
   - 右が速いとき `0.5 + min(0.3, T_lsd / (2 * abs(T_in) + 1e-6))`
   - 左が速いとき `0.5 - min(0.3, T_lsd / (2 * abs(T_in) + 1e-6))`
   - 最後に `0.2` から `0.8` へ制限
8. 効率を掛けた総出力トルクを `T_out = T_in * efficiency` とします。
9. `T_L = T_out * torque_bias`、`T_R = T_out * (1 - torque_bias)` で左右へ分配します。

## しきい値とフォールバック
- 左右速度差が `0.01 rad/s` 未満なら必ず均等配分です。
- 分配比のずれ幅は `±0.3` までで、最終的には `0.2` から `0.8` に収まります。
- 分母ゼロ回避のため `2 * abs(T_in) + 1e-6` を使います。

## 特徴
- クラッチ枚数や差動制限の履歴は持ちません。
- 入力トルクの符号で加速側と減速側ランプを切り替えます。
- 左右の遅い車輪へより多く配分するように書かれています。

## 保存物・描画
保存処理も描画処理もありません。
