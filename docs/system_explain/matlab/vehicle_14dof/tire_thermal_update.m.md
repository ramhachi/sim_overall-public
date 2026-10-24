# tire_thermal_update.m

## 目的
`TIRE_THERMAL_UPDATE` は、タイヤ表面温度を熱収支モデルで 1 ステップ更新する関数です。摩擦仕事による発熱と、風速依存の対流冷却だけを扱います。

## 入口とシグネチャ
```matlab
function T_surf_new = tire_thermal_update(T_surf, Fx, Fy, vsx, vsy, v_wind, T_amb, dt)
```

### 入力
- `T_surf`: 現在の表面温度 `[°C]`
- `Fx`: 前後力 `[N]`
- `Fy`: 横力 `[N]`
- `vsx`: 縦滑り速度 `[m/s]`
- `vsy`: 横滑り速度 `[m/s]`
- `v_wind`: 風速 `[m/s]`
- `T_amb`: 外気温 `[°C]`
- `dt`: タイムステップ `[s]`

### 出力
- `T_surf_new`: 更新後表面温度 `[°C]`

## 外部依存
外部関数やローカル関数はありません。

## 物理モデル
コメントにある支配式は次です。
```matlab
m_tread * c_p * dT/dt = Q_gen - Q_conv - Q_cond
```
ただし実装で実際に計算しているのは `Q_gen` と `Q_conv` のみで、`Q_cond` はコードに現れません。

## 定数
- `m_tread = 2.0` kg
- `c_p = 1000.0` J/(kg·K)
- `h_conv = 50.0` W/(m²·K)
- `A_tread = 0.05` m²

## 処理の流れ
1. 発熱量を
   ```matlab
   Q_gen = abs(Fx * vsx) + abs(Fy * vsy)
   ```
   で計算します。
2. 対流冷却を
   ```matlab
   Q_conv = h_conv * A_tread * (T_surf - T_amb) * (1 + 0.1 * abs(v_wind))
   ```
   で計算します。
3. 温度変化率を
   ```matlab
   dT_dt = (Q_gen - Q_conv) / (m_tread * c_p)
   ```
   とします。
4. オイラー法で
   ```matlab
   T_surf_new = T_surf + dT_dt * dt
   ```
   を計算します。
5. 最後に `0.0` から `120.0` の範囲へクリップします。

## 仮定
- 発熱は前後力と横力の摩擦仕事率の絶対値和です。
- 冷却は対流のみで、風速依存係数は `1 + 0.1 * abs(v_wind)` です。
- 温度は摂氏で計算され、差分もそのまま扱われます。

## フォールバック・制限
- 温度は必ず `0` 以上 `120` 以下に制限されます。
- 熱伝導 `Q_cond` はコメントにはありますが、計算されません。

## 保存物・描画
保存処理も描画処理もありません。
