# `tire_pinn/main_pinn_train.py`

## 1. 役割

PINN タイヤモデルの **学習・評価・デプロイ判定・C++ ランタイム資産出力** を一括で担当する標準 CLI 入口です。

## 2. 入出力

- 実行例:
  - `python3 tire_pinn/main_pinn_train.py --list-tires`
  - `python3 tire_pinn/main_pinn_train.py --tire-model Hoosier_18p0X6p0-10_R20`
  - `python3 tire_pinn/main_pinn_train.py --eval-only`
- 主な入力:
  - `tire_pinn/config/pinn_config.yaml`
  - TTC CSV / raw `.dat`
  - 既存 checkpoint（`--resume`, `--init-checkpoint`）
- 主な出力:
  - `output/tire_models/<tire>/training_artifacts/*.pt`
  - `validation_summary.json`
  - `candidate_results.json`
  - deployable 時は `pinn_runtime.json`, `manifest.json` など

## 3. `main()` の流れ

1. CLI 引数と YAML 設定を読みます。
2. `TireCatalog` から対象タイヤを解決し、必要ならタイヤ別 override を適用します。
3. `TTCDataLoader` でデータと正規化係数を準備します。
4. `TirePINN -> NormalizedTirePINN` を構築し、`CollocationSampler` と `PINNTrainer` を初期化します。
5. `trainer.train()` で学習、または `--eval-only` で checkpoint 評価だけを行います。
6. `best_r2_model.pt / best_model.pt / final_model.pt` を順に評価し、R² と物理試験で最良候補を選びます。
7. deployable 判定を通過した場合だけ `CPPExporter.export()` で実行資産を書き出します。

## 4. 主要関数

### `load_config(config_path)`
- YAML を辞書として返します。

### `deep_merge_dict(base, overrides)`
- ネスト辞書を破壊的にマージします。
- タイヤ別設定上書きの基盤です。

### `apply_tire_profile_overrides(config, tire_name)`
- `tire_profiles.<tire_name>` があれば deep copy 後に上書きします。
- 戻り値: `(マージ済み設定, 適用した override または None)`

### `resolve_device(requested)`
- `auto` / `cuda` / `mps` / `cpu` を解決します。
- 指定デバイスが使えない場合は `RuntimeError` を投げます。

### `load_checkpoint_into_model(model, checkpoint_path, device)`
- checkpoint から `model_state_dict` を復元します。
- optimizer 状態は扱いません。

### `build_runtime_model(model, config)`
- `runtime_postprocess.enabled` が真なら `PhysicsEnforcedTireModel` でラップします。
- 学習モデルとランタイムモデルを分離している点が重要です。

### `evaluate_checkpoint(...)`
- テストローダで推論し、`R2Score` と `PhysicsValidator` を実行します。
- 戻り値は checkpoint 名、パス、ランタイムモデル、検証要約をまとめた辞書です。

### `summarize_validation_results(...)`
- データ完全性、Fx/Fy の最小 R²、物理試験の合否から deployable を判定します。
- 非 deployable 理由も配列で残します。

### `resolve_training_exit_code(validation_summary, allow_nondeployable)`
- 非 deployable は既定で exit 1 です。探索実行だけ `--allow-nondeployable` で exit 0 を許可できます。

## 5. データフローとシステム適合

`TireCatalog` → `TTCDataLoader` → `PINNTrainer` → `PhysicsValidator` → `CPPExporter` という、  
`tire_pinn` サブシステム全体のオーケストレータです。  
学習成果を「研究用 checkpoint」で終わらせず、「シミュレータへ出せる実行資産」まで昇格させるゲートを持ちます。

## 6. 副作用と注意点

- `training_artifacts` と runtime 出力ディレクトリを作成します。
- JSON サマリを書き出します。
- deployable でない場合は runtime export を意図的にスキップします。
