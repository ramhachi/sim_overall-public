# `tire_pinn/export/cpp_exporter.py`

## 1. 役割

学習済み PINN を **C++ 実行系で再現できる JSON 資産一式** へ変換するエクスポータです。

## 2. 出力ファイル

- `pinn_architecture.json`
- `pinn_weights.json`
- `pinn_normalization.json`
- `pinn_runtime.json`
- `manifest.json`
- `test_vectors.json`
- 併せて親ディレクトリの `registry.json` も更新します。

## 3. クラス `CPPExporter`

### `__init__(model, output_dir)`
- モデルと出力先を保持し、出力ディレクトリを作成します。

### `_base_model()`
- `NormalizedTirePINN` 本体を取り出します。
- `PhysicsEnforcedTireModel` のようなラッパ越しでも `base_model` を辿ります。

### `_postprocess_config()`
- モデルが `get_runtime_postprocess_config()` を持つときだけ、ランタイム後処理設定を取得します。

### `export(tire_name, dataset_info, validation_summary, training_summary, input_bounds=None)`
- 全 JSON を順に生成する主入口です。
- manifest を書き、registry 更新とテストベクトル生成まで行います。

### `_export_architecture(path)`
- 線形層・LayerNorm を列挙し、アーキテクチャ情報を書きます。

### `_export_weights(path)`
- 全パラメータを flatten し、C++ 側で読みやすい数値文字列配列へ変換します。

### `_export_normalization(path)`
- 入出力の平均・標準偏差・列名を出力します。

### `_export_runtime_model(path, input_bounds=None)`
- `Sequential` を C++ 側が解釈しやすい layer 配列へ落とし込みます。
- Swish/LayerNorm/Linear を runtime 形式に変換し、必要なら `postprocess` と `input_bounds` を埋め込みます。

### `_build_manifest(...)`
- タイヤ名、データセット、検証結果、学習結果、出力ファイル対応を 1 つの manifest にまとめます。

### `_update_registry(manifest)`
- 親ディレクトリ `registry.json` の対象モデル項目を更新します。

### `export_test_vectors(...)`
- ランダムと代表ケースから入力を作り、モデル出力付きベクトル集を保存します。

### `_generate_test_inputs(...)`
- 固定ケース + 乱数ケースを生成し、`input_bounds` があればクリップします。

## 4. データフローとシステム適合

`main_pinn_train.py` の deployable 判定通過後に呼ばれ、  
PyTorch モデルをシミュレータ実装へ橋渡しする最終段です。  
研究用 checkpoint をランタイム資産へ翻訳する責務を持ちます。

## 5. 副作用と注意点

- 多数の JSON をファイル書き込みします。
- 既存 `registry.json` があれば読み込んで更新します。
- 未対応レイヤがあると `ValueError` を投げます。
