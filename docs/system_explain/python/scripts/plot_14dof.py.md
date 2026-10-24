# `python/scripts/plot_14dof.py`

## 1. 役割

14DOF 車両シミュレーションの CSV 結果を読み、**ステップステアまたは定常円旋回の典型プロット** を出力する可視化スクリプトです。

## 2. 入出力

- 実行例: `python3 python/scripts/plot_14dof.py <result.csv> [output_dir]`
- 入力: `time`, `x`, `y`, `psi`, `vx`, `vy`, `r`, `ay`, `steering` などを持つ CSV
- 出力:
  - `step_steer_plot.png` または `steady_circle_plot.png`
  - 標準出力のサマリー

## 3. 主要制御フロー

1. `main()` が CSV パスから試験種別を推定します。
2. `load_data()` が CSV を読みます。
3. `print_summary()` が継続時間、最終位置、最大横 G などを表示します。
4. 種別に応じて `plot_step_steer()` か `plot_steady_circle()` を実行します。

## 4. 関数

### `load_data(csv_path: str) -> pd.DataFrame`
- CSV を読み込んで DataFrame を返します。
- 副作用: 行数を表示します。

### `plot_step_steer(df, output_dir=None)`
- 役割: ステップステア向け 6 分割図を作ります。
- 図の内容:
  - 軌跡
  - ヨー角
  - `Vx/Vy`
  - ヨーレート
  - 横加速度
  - ステア入力
- 副作用: 出力ディレクトリ生成、PNG 保存、`plt.show()`
- 戻り値: `matplotlib.figure.Figure`

### `plot_steady_circle(df, output_dir=None)`
- 役割: 定常円旋回向け 4 分割図を作ります。
- 図の内容:
  - 円旋回軌跡
  - 速度
  - ヨーレート
  - 横加速度

### `print_summary(df, test_type: str)`
- 役割: 試験の時間長、最終位置・速度、最大横 G、最大ヨーレートを表示します。
- 副作用: 標準出力のみ。

### `main()`
- 役割: CLI 入口です。
- 注意: ファイル名に `step_steer` を含むかどうかで種別判定しており、明示オプションではありません。
