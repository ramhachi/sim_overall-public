# `tire_fitting/main.py`

## 役割

TMeasy ベースのタイヤフィッティング CLI です。  
説明文や import には hybrid 系が見えますが、実際の本処理は `TMeasyModel` と `TireParameterOptimizer` を使う **単独モデルの 5 段パイプライン** です。

## 実行と入出力

- 実行例:
  - `python tire_fitting/main.py --drivebrake <csv> --cornering <csv> --output <dir> --tire-name <name>`
- 主な入力:
  - DriveBrake / Cornering ファイル
  - 出力ディレクトリ
  - タイヤ名、公称荷重、有効半径
  - 外れ値除去フラグ、設定 YAML
- 主な出力:
  - `<output>/<tire>.yaml`
  - `<output>/<tire>.hpp`
  - `fx_vs_sr.png`, `fy_vs_sa.png`, `residuals_fx.png`, `residuals_fy.png`
- 戻り値:
  - `main()` は終了コード `0/1`

## 主要関数

### `load_config_file(config_path)`
- YAML 設定を読み、`model_long` / `model_lat` の妥当性を検査します。
- 現在のメイン処理ではそれらの値は実質未使用ですが、設定読込の口は残っています。

### `merge_config_with_args(args)`
- CLI の既定値が残っている場合だけ設定ファイル値を適用します。
- CLI 優先のマージです。

### `parse_arguments()`
- 必須引数は `--drivebrake`, `--cornering`, `--output`, `--tire-name` です。
- 任意で半径、荷重、フィルタ閾値、外れ値除去、設定ファイルを受けます。

### `main()`
- 全体制御です。
- 主処理は以下の 5 段です。
  1. `TireDataLoader` で入出力データ読込
  2. 欠損除去・`FZ>0`・縦力用 `|SA|<5deg` フィルタ
  3. `TireParameterOptimizer` で縦横パラメータ推定
  4. `FitEvaluator` で指標算出、ピーク誤差確認、PNG 生成
  5. `ParameterExporter` で YAML / C++ 出力
- 途中で目標精度 (`RMSE<100`, `TIC<0.15`, `peak error<5%`) も判定します。

## データフロー

1. TTC データを読み込み、縦力用と横力用に必要列を抽出します。
2. 縦力は `SR/FX/FZ`、横力は `SA/FY/FZ/IA` を `numpy` 配列へ変換します。
3. 最適化で `mu_x_peak`, `k_mu_x`, `Kx`, `mu_y_peak`, `k_mu_y`, `Ky`, `C_gamma` を推定します。
4. 同じモデルで予測値を再計算し、指標・図・精度判定を作ります。
5. 旧形式 YAML / C++ ヘッダに集約して保存します。

## システム内での位置づけ

`main_hybrid.py` より古い **TMeasy 単独の標準入口** です。  
hybrid 化の import は含みますが、現状の処理経路は旧 `exporter.py` / `tmeasy_model.py` / `optimizer.TireParameterOptimizer` に接続されています。
