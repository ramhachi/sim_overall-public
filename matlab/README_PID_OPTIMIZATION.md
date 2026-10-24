# PIDゲイン最適化ガイド

## 🎯 概要

PIDドライバーのゲインを自動的に最適化して、車両の経路追従性能を向上させます。

**2つの最適化手法：**
1. **グリッドサーチ** - 高速（10-15分）、主要ゲインのみ
2. **ベイズ最適化** - 高精度（20-40分）、全ゲイン探索

---

## 🚀 クイックスタート

### 方法1: グリッドサーチ（推奨・初回）

```matlab
% MATLABで実行
cd /Users/sota/projects/FomulaTMU/sim_overall/matlab

% 8の字トラックで最適化
best_gains = grid_search_pid('figure_eight', 25, 10)

% 結果を確認
% - grid_optimized.csv（最適化後のシミュレーション結果）
% - grid_search_results_figure_eight.txt（最適ゲイン）
```

**所要時間：** 約10-15分
**探索範囲：** Kp_steer, Kd_steer, Kh_steer（主要3ゲイン）

### 方法2: ベイズ最適化（高精度）

```matlab
% MATLABで実行
cd /Users/sota/projects/FomulaTMU/sim_overall/matlab

% より詳細な最適化
results = optimize_pid_gains('figure_eight', 25, 15)

% 結果を確認
% - optimized_result.csv（最適化後のシミュレーション結果）
% - optimal_gains_figure_eight.txt（最適ゲイン）
```

**所要時間：** 約20-40分（MaxObjectiveEvaluations=50）
**探索範囲：** 全7ゲイン（Kp/Kd/Ki/Kh/Kff_steer, Kp/Ki_speed）

---

## 📊 評価指標（コスト関数）

最適化では以下を最小化します：

```
Cost = 10.0 × RMS(横偏差)
     + 0.5 × RMS(ヘディング誤差)
     + 1.0 × RMS(速度誤差)
     + 2.0 × ステアリング変化量
     + 0.5 × スロットル変化量
     + 5.0 × 最大横偏差
     + 0.2 × 最大ヘディング誤差
```

**重視する項目：**
- 横偏差（トラック追従精度）
- ステアリングの滑らかさ（車両安定性）
- 最大誤差（安全マージン）

---

## 🎛️ PIDゲインの説明

### ステアリング制御

| ゲイン | 説明 | デフォルト | 最適化範囲 |
|--------|------|-----------|-----------|
| `Kp_steer` | 横偏差に対する比例ゲイン（最重要） | 0.8 | 0.1 - 2.0 |
| `Kd_steer` | 横偏差変化率に対する微分ゲイン | 0.3 | 0.05 - 1.0 |
| `Ki_steer` | 横偏差累積に対する積分ゲイン | 0.05 | 0.0 - 0.2 |
| `Kh_steer` | ヘディング誤差に対するゲイン | 0.5 | 0.1 - 1.5 |
| `Kff_steer` | 曲率フィードフォワードゲイン | 1.0 | 0.5 - 2.0 |

### 速度制御

| ゲイン | 説明 | デフォルト | 最適化範囲 |
|--------|------|-----------|-----------|
| `Kp_speed` | 速度誤差に対する比例ゲイン | 0.3 | 0.1 - 1.0 |
| `Ki_speed` | 速度誤差累積に対する積分ゲイン | 0.05 | 0.0 - 0.3 |

---

## 📝 最適化結果の使い方

### 1. テキストファイルから確認

```bash
cat optimal_gains_figure_eight.txt
```

出力例：
```
=== Optimal PID Gains for figure_eight Track ===
Track radius: 25.0 m

# C++ Command Line Arguments:
--kp-steer 1.052341 --kd-steer 0.384567 --ki-steer 0.078901 \
--kh-steer 0.623456 --kff-steer 1.234567 \
--kp-speed 0.345678 --ki-speed 0.089012
```

### 2. C++シミュレーションで使用

```bash
docker compose run --rm fsae-sim bash -c "cd build && \
  ./cpp/laptime_sim --track figure_eight --radius 25 --time 20 \
  --kp-steer 1.052341 --kd-steer 0.384567 --ki-steer 0.078901 \
  --kh-steer 0.623456 --kff-steer 1.234567 \
  --kp-speed 0.345678 --ki-speed 0.089012"
```

### 3. プリセットとして登録（オプション）

`cpp/include/fsae/driver/pid_driver.hpp`に追加：

```cpp
/// 最適化済みゲイン（8の字トラック）
inline PIDGains optimized_figure_eight_gains() {
    PIDGains gains;
    gains.Kp_steer = 1.052341;
    gains.Kd_steer = 0.384567;
    gains.Ki_steer = 0.078901;
    gains.Kh_steer = 0.623456;
    gains.Kff_steer = 1.234567;
    gains.Kp_speed = 0.345678;
    gains.Ki_speed = 0.089012;
    return gains;
}
```

---

## 🔧 カスタマイズ

### 探索範囲の変更

`optimize_pid_gains.m`内で範囲を調整：

```matlab
Kp_steer = optimizableVariable('Kp_steer', [0.5, 1.5], 'Type', 'real');  % 範囲を狭める
```

### 評価回数の変更

```matlab
results = bayesopt(@objective_function, vars, ...
    'MaxObjectiveEvaluations', 100, ...  % 50 → 100に増やす
    ...
```

### コスト関数の重みを変更

`optimize_pid_gains.m`の`objective_function`内：

```matlab
w_lat = 20.0;      % 横偏差の重みを増やす（デフォルト: 10.0）
w_smooth_s = 1.0;  % ステアリング滑らかさの重みを減らす（デフォルト: 2.0）
```

---

## 🎨 最適化の可視化

最適化中、MATLABが自動的にプロットを表示：

1. **Objective Function Model**: ベイズモデルの予測
2. **Min Objective**: 最小コストの推移

グリッドサーチでは：
- **Cost Map**: ゲイン空間でのコストヒートマップ
- **Error Trade-off**: 横偏差 vs ヘディング誤差
- **Cost History**: 評価回数とコストの推移

---

## 🐛 トラブルシューティング

### エラー: シミュレーションが不安定

**原因**: ゲインが極端すぎて車両が発散
**対策**: 探索範囲を狭める、シミュレーション時間を短くする

### エラー: Docker Desktopが起動していない

```
Error response from daemon: ...
```

**対策**: Docker Desktopを起動してから再実行

### 最適化が遅い

**対策1**: グリッドサーチを使う（ベイズ最適化より高速）
**対策2**: シミュレーション時間を短くする（15s → 10s）
**対策3**: MaxObjectiveEvaluationsを減らす（50 → 30）

### 結果が期待より悪い

**原因**: 評価関数が目的に合っていない
**対策**: コスト関数の重みを調整

---

## 📚 関連ファイル

- `optimize_pid_gains.m` - ベイズ最適化スクリプト
- `grid_search_pid.m` - グリッドサーチスクリプト
- `visualize_laptime_sim.m` - 結果可視化
- `cpp/src/main_laptime_sim.cpp` - シミュレータ本体
- `cpp/include/fsae/driver/pid_driver.hpp` - PIDコントローラー

---

## 💡 Tips

1. **最初はグリッドサーチ** - 大まかな最適範囲を把握
2. **ベイズ最適化で微調整** - グリッド結果の周辺を集中探索
3. **トラック別に最適化** - circle, figure_eight, straight_cornerで異なる特性
4. **複数回実行** - ベイズ最適化は確率的なので、最良結果を選ぶ
5. **シミュレーション時間は適度に** - 長すぎると遅い、短すぎると不正確

---

## 🎯 次のステップ

最適化後：
1. ✅ 最適ゲインで長時間シミュレーション実行
2. ✅ 他のトラックでも最適化
3. ✅ 実車両パラメータで
4. 検証
5. ✅ より高度な制御手法を検討（MPC, Stanley制御など）
