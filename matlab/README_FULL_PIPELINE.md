# フルパイプライン自動実行ガイド

## 概要

`run_full_pipeline.m` は、PID最適化→シミュレーション実行→結果可視化を**ワンクリック**で実行するMATLABスクリプトです。

## クイックスタート

### 1. 最もシンプルな実行（デフォルトゲイン）

```matlab
% MATLABで実行
cd /Users/sota/projects/FomulaTMU/sim_overall/matlab
run_full_pipeline('circle', 30)
```

**実行内容：**
- 円形トラック（半径30m）
- デフォルトPIDゲイン
- 40秒シミュレーション
- 自動可視化

**所要時間：** 約10-20秒

---

### 2. PID最適化を含む完全実行（推奨）

```matlab
% 円形トラック、ベイズ最適化30回
run_full_pipeline('circle', 30, 'optimize', true, 'n_iter', 30)
```

**実行内容：**
1. **Phase 1**: ベイズ最適化（30回試行）
2. **Phase 2**: 最適化されたゲインでシミュレーション
3. **Phase 3**: 結果可視化
4. **Phase 4**: レポート生成

**所要時間：** 約15-30分（最適化による）

---

### 3. グリッドサーチで高速最適化

```matlab
% グリッドサーチ（ベイズより高速）
run_full_pipeline('circle', 30, 'optimize', true, 'method', 'grid')
```

**所要時間：** 約10-15分

---

## 詳細オプション

### パラメータ一覧

| パラメータ | 説明 | デフォルト |
|-----------|------|-----------|
| `track_type` | トラックタイプ (`'circle'`, `'figure_eight'`, `'straight_corner'`) | - |
| `radius` | トラック半径 [m] | - |
| `'optimize'` | PID最適化を実行するか | `false` |
| `'method'` | 最適化手法 (`'bayesian'` or `'grid'`) | `'bayesian'` |
| `'n_iter'` | ベイズ最適化の反復回数 | `30` |
| `'sim_time'` | シミュレーション時間 [s] | `40.0` |
| `'initial_speed'` | 初速 [m/s] | `8.0` |
| `'save_report'` | レポート保存 | `true` |

### 実行例

#### 例1: 円形トラック、50回最適化、60秒シミュレーション

```matlab
run_full_pipeline('circle', 30, ...
    'optimize', true, ...
    'n_iter', 50, ...
    'sim_time', 60.0)
```

#### 例2: 8の字トラック、グリッドサーチ、初速10m/s

```matlab
run_full_pipeline('figure_eight', 25, ...
    'optimize', true, ...
    'method', 'grid', ...
    'initial_speed', 10.0)
```

#### 例3: カスタムトラック、最適化なし

```matlab
run_full_pipeline('straight_corner', 20, ...
    'sim_time', 30.0, ...
    'initial_speed', 12.0)
```

---

## 出力ファイル

### 自動生成されるファイル

1. **CSV データ** (`laptime_sim_<track>.csv`)
   - 時系列データ（位置、速度、制御入力など）
   - 後から再可視化可能

2. **最適化結果** (`optimal_gains_<track>.txt` or `grid_search_results_<track>.txt`)
   - 最適PIDゲイン（コマンドライン形式）
   - 次回の実行で再利用可能

3. **レポート** (`report_<track>_<timestamp>.txt`)
   - 実行設定、結果サマリー、ファイルリスト
   - 実験記録として保存

4. **MATLABフィギュア** （自動表示）
   - Figure 1: メイン結果（軌跡、速度、制御入力）
   - Figure 2: 詳細時系列
   - Figure 3: g-gダイアグラム

---

## トラブルシューティング

### 問題1: Docker Desktopが起動していない

**エラー：**
```
Cannot connect to the Docker daemon
```

**解決策：**
Docker Desktopを起動してから再実行してください。

---

### 問題2: MATLABディレクトリ以外から実行

**警告：**
```
MATLABディレクトリから実行してください
```

**解決策：**
```matlab
cd /Users/sota/projects/FomulaTMU/sim_overall/matlab
```

---

### 問題3: シミュレーションが失敗する

**原因：**
- 極端なPIDゲインで車両が不安定化
- ビルドエラー

**解決策：**
```matlab
% デフォルトゲインで試す
run_full_pipeline('circle', 30)
```

---

### 問題4: 最適化が遅すぎる

**解決策1: グリッドサーチに変更**
```matlab
run_full_pipeline('circle', 30, 'optimize', true, 'method', 'grid')
```

**解決策2: 反復回数を減らす**
```matlab
run_full_pipeline('circle', 30, 'optimize', true, 'n_iter', 10)
```

---

## ワークフロー例

### 研究開発フロー

1. **初回実行（デフォルトゲイン）**
   ```matlab
   run_full_pipeline('circle', 30)
   ```
   → 結果を確認、問題を特定

2. **PID最適化実行**
   ```matlab
   run_full_pipeline('circle', 30, 'optimize', true, 'n_iter', 30)
   ```
   → 最適ゲインを取得

3. **長時間シミュレーション**
   ```matlab
   run_full_pipeline('circle', 30, 'sim_time', 120.0)
   ```
   → 2周分のデータで性能評価

4. **トラック変更**
   ```matlab
   run_full_pipeline('figure_eight', 25, 'optimize', true)
   ```
   → 異なるトラックで検証

---

## 高度な使い方

### 最適化結果の再利用

最適化は一度実行すれば、次回は結果を再利用できます：

```matlab
% 1回目：最適化を実行
run_full_pipeline('circle', 30, 'optimize', true, 'n_iter', 50)

% 2回目以降：最適化をスキップ（optimal_gains_circle.txtが使用される）
run_full_pipeline('circle', 30, 'sim_time', 60.0)
```

**注意：** 手動で最適ゲインを使うには、`laptime_sim`を直接呼び出してください。

---

### バッチ実行

複数の半径でテストする場合：

```matlab
radii = [20, 25, 30, 35, 40];
for r = radii
    run_full_pipeline('circle', r, 'optimize', true, 'method', 'grid')
    pause(5);  % 結果を確認する時間
end
```

---

### カスタム可視化

自動生成された Figure を保存する場合：

```matlab
run_full_pipeline('circle', 30)

% Figure 1を保存
figure(1);
saveas(gcf, 'trajectory.png');

% Figure 3（g-g図）を保存
figure(3);
saveas(gcf, 'gg_diagram.png');
```

---

## 関連ドキュメント

- **PID最適化詳細**: `README_PID_OPTIMIZATION.md`
- **可視化詳細**: `README_VISUALIZATION.md`
- **開発ログ**: `../docs/DEVELOPMENT_LOG_PHASE7.md`
- **総合ガイド**: `../学生フォーミュラ車両運動シミュレーション総合ガイド.md`

---

## コマンドリファレンス

### 基本コマンド

```matlab
% デフォルト実行
run_full_pipeline('circle', 30)

% 最適化付き
run_full_pipeline('circle', 30, 'optimize', true)

% グリッドサーチ
run_full_pipeline('circle', 30, 'optimize', true, 'method', 'grid')

% カスタム時間・速度
run_full_pipeline('circle', 30, 'sim_time', 60, 'initial_speed', 12)
```

### ヘルプ表示

```matlab
help run_full_pipeline
```

---

## FAQ

**Q1: どの最適化手法を選ぶべきですか？**

A:
- **ベイズ最適化**：高精度、時間がかかる（20-40分）
- **グリッドサーチ**：中精度、高速（10-15分）

初めての場合はグリッドサーチを推奨します。

**Q2: 最適化は毎回実行すべきですか？**

A: いいえ。同じトラックタイプ・半径なら、一度最適化すれば結果を再利用できます。

**Q3: 8の字トラックは使えますか？**

A: 現在の実装では8の字トラック（自己交差）は不安定です。円形トラックを推奨します。

**Q4: 結果をどう評価すべきですか？**

A: 以下の指標を確認してください：
- ラップ完了の有無
- 横偏差（平均・最大）がトラック幅以内か
- 速度が安定しているか

**Q5: エラーが出たらどうすればいいですか？**

A:
1. Docker Desktopが起動しているか確認
2. デフォルトゲインで実行してみる
3. 開発ログのトラブルシューティングを参照

---

**Happy Simulating! 🏎️**
