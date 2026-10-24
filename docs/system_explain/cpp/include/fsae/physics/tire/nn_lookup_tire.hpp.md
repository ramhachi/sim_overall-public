# nn_lookup_tire.hpp

## ファイルの目的
- JSON またはテキスト書き出し済み PINN/NN タイヤモデルを読み込み、推論できる C++ 実行時ローダを提供します。

## 実行上の役割
- 学習済みモデルの正規化、線形層、LayerNorm、活性化、物理後処理を実車シミュレーション側で再現します。
- `Magic Formula` や `TMeasy` と共通に扱える `TireForces` を返す実行器です。

## 依存関係
- 外部: `<array>`, `<algorithm>`, `<cmath>`, `<filesystem>`, `<fstream>`, `<iostream>`, `nlohmann/json.hpp`, `<sstream>`, `<stdexcept>`, `<string>`, `<utility>`, `<vector>`
- リポジトリ内依存: なし（このファイルが `TireForces` の定義元）

## 入力/出力
- 入力: `TireInput{SR,SA,Fz,IA,TSTC}` と、エクスポート済みモデルディレクトリ/ファイル
- 出力: `TireForces{Fx,Fy,Mx,Mz}`

## 関数・メソッド・クラスの網羅的な説明
- `struct TireForces`
  - NN・MF・他モデル共通の出力構造体です。
  - `Fx`, `Fy`, `Mx`, `Mz` を保持します。
- `struct TireInput`
  - 入力 5 変数です。
  - `SR` スリップ率、`SA` スリップ角、`Fz` 垂直荷重、`IA` キャンバ、`TSTC` 温度入力です。
- `struct FCLayer`
  - `weights`, `biases`, `n_in`, `n_out` を持つ全結合層です。
  - `forward(const std::vector<double>& input) const`
    - 入力長を検証し、`Wx+b` を返します。
- `struct BatchNormLayer`
  - `gamma`, `beta`, `mean`, `var`, `epsilon`, `n_channels` を持つ旧形式 BatchNorm です。
  - `forward(const std::vector<double>& input) const`
    - 各チャネルを正規化して返します。
- `struct RuntimeLayer`
  - 実行時 JSON 表現の 1 層です。
  - `enum class Type { Linear, LayerNorm, Swish, ReLU }` で層種別を持ちます。
  - `weights`, `bias`, `epsilon` は層パラメータ格納用です。
- `class NNTextTireModel`
  - `NNTextTireModel()`
    - 空モデルを作ります。
  - `load_from_export(const std::string& base_dir)`
    - `manifest.json` を優先し、次に `pinn_runtime.json`、最後に旧式テキスト群 `normalization_params.txt` / `network_weights.txt` / `network_architecture.txt` を読みます。
    - 後処理フラグやゲート係数も初期化します。
  - `load_from_manifest(const std::string& manifest_path)`
    - manifest を読み、非 deployable モデルを拒否し、実体 JSON への相対パスも解決します。
  - `predict(const TireInput& input) const`
    - 入力をベクタ化し、必要なら境界にクランプ、`forward_physical()` で推論し、`apply_runtime_postprocess()` 後に `TireForces` へ詰めます。
  - `is_loaded() const`
    - モデル読込済みか返します。
  - `num_fc_layers() const`
    - runtime モードでは `RuntimeLayer::Linear` の数、旧形式では `fc_layers_` の数を返します。
  - private データ
    - `fc_layers_`, `bn_layers_`, `runtime_layers_` はモデル本体です。
    - `input_mean_/std_/min_/max_`, `output_mean_/std_` は正規化情報です。
    - `has_input_bounds_`, `use_runtime_postprocess_`, `friction_cap_mu_`, `zero_slip_sr_scale_`, `zero_slip_sa_scale_deg_`, `symmetry_sr_scale_`, `symmetry_ia_scale_deg_`, `sa_zero_fy_scale_deg_`, `postprocess_epsilon_`, `loaded_` は後処理設定と状態です。
  - private `static swish(double x)`
    - Swish 活性化です。
  - private `static layer_norm(...)`
    - mean/variance をその場で計算する LayerNorm 実装です。
  - private `forward_runtime(const std::vector<double>& input) const`
    - `runtime_layers_` を順に実行します。
    - `Linear`, `LayerNorm`, `Swish`, `ReLU` のみ対応です。
  - private `forward_physical(const std::vector<double>& physical_input) const`
    - 物理量入力を正規化し、runtime 形式か legacy 形式かで前向き計算し、最後に出力逆正規化します。
  - private `apply_runtime_postprocess(const std::vector<double>& physical_input, std::vector<double>& output) const`
    - ゼロスリップ補正、左右対称投影、`SA→0` 時の `Fy` 漏れ抑制、摩擦円キャップを適用します。
    - ゼロスリップ補正では追加の `forward_physical()` を呼び直して基準出力を差し引きます。
  - private `forward_legacy(const std::vector<double>& input) const`
    - 旧形式 `fc_layers_` + 任意 `bn_layers_` + ReLU で順伝播します。
  - private `load_runtime_json(const std::string& filepath)`
    - `input/output/layers/postprocess/input_bounds` を持つ runtime JSON を読み込み、`runtime_layers_` を組み立てます。
  - private `load_normalization(const std::string& filepath)`
    - テキスト形式の入力/出力平均・標準偏差を読み込みます。
  - private `load_weights(const std::string& filepath)`
    - `LAYER_*_WEIGHTS` / `LAYER_*_BIASES` セクションから旧形式全結合層を読み込みます。
  - private `load_architecture(const std::string& filepath)`
    - 現状は未使用で、`bn_layers_` を空にするだけです。
