# デモ実行手順（Docker版）

## 前提条件

- ✅ Docker Desktop が起動していること
- ✅ Eigen3とnlohmann-jsonがDockerイメージにインストール済み（完了）
- ✅ C++実行ファイルがビルド済み（完了）

## 実行方法

### MATLABから実行

```matlab
cd /Users/sota/projects/FomulaTMU/sim_overall/frame_optimize/examples
demo_simple_frame
```

### 実行される処理

1. **MATLABデータ準備**: 16ノード、16要素のフレームジオメトリ
2. **JSON出力**: `frame_optimize/temp/input_*.json`に入力データを出力
3. **Docker実行**: `docker compose run --rm fsae-sim bash -c "./beam_fem_standalone ..."`
4. **C++計算**: Docker内のFEMソルバーがねじり剛性を計算
5. **JSON入力**: `frame_optimize/temp/output_*.json`から結果を読み込み
6. **結果表示**: 質量、ねじり剛性、最大変位を表示
7. **可視化**: 3Dフレーム形状と変形図を描画

### 期待される出力

```
=== FSAE Frame Optimization Demo ===

1. Loading materials and pipe catalog...
   Loaded 4 materials, 18 pipes

2. Creating simple frame geometry...
   Created 16 nodes, 16 elements

3. Setting up boundary conditions and loads...
   BCs: 12 constraints
   Loads: Applied ±1000 Nm torsion

4. Running MEX FEM solver...
   ✓ Solve completed in ~1000 ms (Docker overhead included)

=== RESULTS ===
Mass:               9.34 kg
Torsional rigidity: 245.6 Nm/deg
Max displacement:   2.134 mm

✓ Demo completed successfully!
```

**注意**: Docker版は初回実行で1-2秒かかります（コンテナ起動のオーバーヘッド）。MEX版（macOS直接実行）なら3-5msですが、今回はXcodeライセンス問題を回避するためDocker版を使用しています。

## トラブルシューティング

### 問題: "Docker daemon is not running"

**解決策**: Docker Desktopを起動してください。

```bash
open -a Docker
```

### 問題: "Cannot read from output file"

**解決策**: Dockerコンテナ内でエラーが発生した可能性があります。手動で確認：

```bash
cd /Users/sota/projects/FomulaTMU/sim_overall/frame_optimize/temp
ls -la
cat output_*.json  # エラーメッセージを確認
```

### 問題: "Solver failed with unknown error"

**解決策**: Docker内で直接実行してエラーを確認：

```bash
docker compose run --rm fsae-sim bash
cd /workspace/frame_optimize/bin
./beam_fem_standalone ../temp/input_*.json ../temp/output_*.json
```

## ファイル構成

```
frame_optimize/
├── bin/
│   └── beam_fem_standalone    # Docker内でビルドされた実行ファイル（281KB）
├── cpp/
│   ├── beam_fem_standalone.cpp  # C++ソースコード
│   ├── CMakeLists.txt
│   └── build/                   # ビルド成果物
├── matlab/
│   └── beam_fem_solver.m        # MATLABラッパー関数（JSON I/O + Docker呼び出し）
├── temp/                        # 一時ファイル（JSON入出力）
│   ├── input_*.json
│   └── output_*.json
└── examples/
    └── demo_simple_frame.m      # デモスクリプト
```

## 次のステップ

Week 1のデリバリーが完了したら、Week 2に進みます：
- `checkFrameCompliance.m` - FSAE規則チェッカー
- `checkExclusionZones.m` - 阻止領域判定
- `frameObjectiveFunction.m` - 統合目的関数

---

**作成日**: 2026-02-24
**方式**: Docker + スタンドアロンC++ + JSON I/O
**理由**: Xcodeライセンス問題を回避するため
