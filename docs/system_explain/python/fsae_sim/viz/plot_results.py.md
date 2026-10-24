# `python/fsae_sim/viz/plot_results.py`

## 1. 役割

シミュレーション CSV を汎用的に可視化するモジュールです。  
単体結果の描画と、MATLAB/C++ 出力の比較描画の 2 つを提供します。

## 2. 入出力

- 入力:
  - 単体結果 CSV
  - または MATLAB/C++ の比較 CSV
- 出力:
  - `vehicle_response.png`
  - `comparison.png`
  - `matplotlib.figure.Figure`

## 3. 関数

### `setup_plot_style()`
- 役割: `matplotlib` の共通スタイルを設定します。
- 副作用: `plt.rcParams` を変更します。

### `plot_vehicle_response(result_file: Path, output_dir: Optional[Path] = None, show: bool = True)`
- 役割: 単一結果 CSV を 3x2 レイアウトで描きます。
- 内容:
  - 軌跡 (`x`,`y`)
  - 速度 (`vx`,`vy`)
  - 姿勢角 (`phi`,`theta`,`psi`)
  - ヨーレート (`r`)
  - タイヤ横力 (`Fy_*`)
  - タイヤ温度 (`T_*`)
- 特徴: 列が存在する時だけ該当サブプロットを描く可変構成です。
- 副作用: PNG 保存、必要に応じて `plt.show()`

### `plot_comparison(matlab_file: Path, cpp_file: Path, output_dir: Optional[Path] = None, show: bool = True)`
- 役割: MATLAB/C++ の状態比較図を作ります。
- 入力前提: ヘッダ無し CSV、先頭列が時間、以降が状態量。
- 比較対象: 最大 4 状態 (`z_s`, `z_u`, `dz_s`, `dz_u`)
- 副作用: `comparison.png` 保存、必要に応じて表示。

## 4. CLI フロー

`__main__` では単体結果描画のみを呼びます。  
比較描画はモジュール関数として他所から使う前提です。
