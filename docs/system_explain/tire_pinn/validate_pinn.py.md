# `tire_pinn/validate_pinn.py`

## 1. 役割

学習済み PINN の **予測対実測比較** と **物理妥当性の可視化** を行う検証スクリプトです。  
Fx/Fy/Mx/Mz の 1:1 散布図に加え、Fy-SA、Fx-SR、ゼロスリップ、摩擦円を PNG 化します。

## 2. 入出力

- 実行例:
  - `python3 tire_pinn/validate_pinn.py`
  - `python3 tire_pinn/validate_pinn.py --checkpoint output/pinn/best_model.pt`
- 主な入力:
  - checkpoint
  - `tire_pinn/config/pinn_config.yaml`
  - TTC データ
- 主な出力:
  - `<config.output.dir>/validation/pred_vs_actual.png`
  - `fy_vs_sa.png`
  - `fx_vs_sr.png`
  - `zero_slip_check.png`
  - `friction_circle.png`

## 3. `main()` の流れ

1. 引数から checkpoint / config / device を読みます。
2. `load_model()` でモデルと `TTCDataLoader` を復元します。
3. `get_test_data()` で test split を取り出します。
4. テスト集合に対する予測値を生成します。
5. 5 種類の plot 関数を順番に実行し、画像を保存します。
6. 最後に `SR=SA=IA=0` での荷重別数値表を標準出力へ出します。

## 4. 関数

### `load_model(config, checkpoint_path, device)`
- `TTCDataLoader(normalize=True)` から正規化係数を取得します。
- `TirePINN -> NormalizedTirePINN` を構築し、checkpoint をロードします。
- `runtime_postprocess.enabled` 時は `PhysicsEnforcedTireModel` を被せます。
- 戻り値: `(model, loader)`

### `get_test_data(loader, config)`
- `split_data()` を呼んで test split の `X_test`, `y_test` を返します。

### `plot_pred_vs_actual(y_pred, y_test, output_dir)`
- 4 出力の 1:1 散布図を 2x2 で描き、R² と RMSE を注記します。

### `plot_fy_vs_sa(model, loader, device, output_dir)`
- `SR=0` で SA を掃引し、荷重別 Fy 曲線を描きます。
- 実 TTC データの `|SR|<0.02` 散布図も重ねます。

### `plot_fx_vs_sr(model, loader, device, output_dir)`
- `SA=0` で SR を掃引し、荷重別 Fx 曲線を描きます。
- 実データ側は `|SA|<1°` に絞ります。

### `plot_zero_slip_check(model, loader, device, output_dir)`
- `SR=SA=IA=0` における Fx/Fy/Mx/Mz の荷重依存を確認します。
- 各パネルに最大絶対値を表示します。

### `plot_friction_circle(model, loader, device, output_dir)`
- 固定荷重で SR×SA 格子を掃引し、Fx-Fy 分布と摩擦円上限を重ねます。

## 5. システム内の位置付け

本番デプロイ判定そのものは `main_pinn_train.py` 側で行いますが、  
このファイルは「なぜ通った/通らないか」を人間が目視で理解する補助検証器です。

## 6. 副作用

- 出力ディレクトリを作成します。
- 複数の PNG を保存します。
- 数値要約を標準出力へ表示します。
