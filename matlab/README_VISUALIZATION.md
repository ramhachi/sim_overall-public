# ラップタイムシミュレーション可視化ガイド

## クイックスタート（8の字トラック）

### 1. シミュレーション実行（ターミナル）

```bash
cd /Users/sota/projects/FomulaTMU/sim_overall

# Docker環境でシミュレーション実行
docker compose run --rm fsae-sim bash -c "cd build && ./cpp/laptime_sim --track figure_eight --radius 25 --time 20"

# 結果をMATLABディレクトリにコピー
docker compose run --rm fsae-sim bash -c "cp build/laptime_sim_figure_eight.csv /workspace/matlab/"
```

### 2. MATLAB可視化

```matlab
% MATLABを起動
cd /Users/sota/projects/FomulaTMU/sim_overall/matlab

% 可視化スクリプト実行
run_visualization
```

これで3つのフィギュアが表示されます：
- **Figure 1**: メイン結果（軌跡、速度、誤差、制御入力）
- **Figure 2**: 詳細時系列データ
- **Figure 3**: g-gダイアグラム

## ワンライナー実行

### ターミナル（シミュレーション + コピー）
```bash
docker compose run --rm fsae-sim bash -c "cd build && ./cpp/laptime_sim --track figure_eight --radius 25 --time 20 && cp laptime_sim_figure_eight.csv /workspace/matlab/"
```

### MATLAB（可視化のみ）
```matlab
cd /Users/sota/projects/FomulaTMU/sim_overall/matlab; run_visualization
```

## カスタムシミュレーション

### 他のトラックタイプ

**円形トラック：**
```bash
docker compose run --rm fsae-sim bash -c "cd build && ./cpp/laptime_sim --track circle --radius 30 --time 20 && cp laptime_sim_circle.csv /workspace/matlab/"
```
```matlab
visualize_laptime_sim('laptime_sim_circle.csv', 'circle', 30)
```

**直線+コーナー：**
```bash
docker compose run --rm fsae-sim bash -c "cd build && ./cpp/laptime_sim --track straight_corner --radius 15 --time 15 && cp laptime_sim_straight_corner.csv /workspace/matlab/"
```
```matlab
visualize_laptime_sim('laptime_sim_straight_corner.csv', 'straight_corner', 15)
```

### パラメータ調整

```bash
# PIDゲインを直接指定
docker compose run --rm fsae-sim bash -c "cd build && ./cpp/laptime_sim --track figure_eight --kp-steer 2.0 --kd-steer 0.2 --time 20"

# 初速変更
docker compose run --rm fsae-sim bash -c "cd build && ./cpp/laptime_sim --track figure_eight --initial-speed 15 --time 20"

# オプション一覧はソースの引数解析を参照
rg -n 'arg == "--' cpp/src/main_laptime_sim.cpp
```

## ファイル構成

- `visualize_laptime_sim.m` - メイン可視化関数
- `run_visualization.m` - クイック実行スクリプト（8の字）
- `laptime_sim_*.csv` - シミュレーション結果データ

## トラブルシューティング

**Q: CSVファイルが見つかりません**
```
A: シミュレーション実行後、CSVをmatlabディレクトリにコピーしてください：
   docker compose run --rm fsae-sim bash -c "cp build/laptime_sim_*.csv /workspace/matlab/"
```

**Q: フィギュアが表示されない**
```
A: MATLABのカレントディレクトリが matlab/ であることを確認してください：
   pwd  % 現在のディレクトリを確認
   cd /Users/sota/projects/FomulaTMU/sim_overall/matlab
```

**Q: エラー: 変数に割り当てられた値は使用されていない**
```
A: これは警告です。プログラムは正常に動作します。
```

## 可視化内容

### Figure 1: メイン結果
- 🗺️ **車両軌跡**: トラック中心線 + 実軌跡（速度で色分け）
- 📈 **速度プロファイル**: 実速度 vs 目標速度
- 📏 **横偏差**: トラック中心線からの距離誤差
- 🧭 **ヘディング誤差**: 進行方向の角度誤差
- 🎯 **ステアリング角**: 操舵入力
- ⚡ **スロットル/ブレーキ**: 加減速入力

### Figure 2: 詳細時系列
- 速度成分（vx, vy）
- ヨーレート（回転角速度）
- 弧長（トラック上の走行距離）
- 曲率（トラックの曲がり具合）
- 速度誤差
- 横偏差 vs 距離

### Figure 3: g-gダイアグラム
- 加速度プロット（ax vs ay）
- 理論限界線
  - 横: 1.5G
  - 縦加速: 1.0G
  - 制動: 1.8G
