# `cpp/tests/test_magic_formula.cpp`

## 1. ファイルの目的

Magic Formula タイヤモデルの純縦力・純横力・複合スリップ・セルフアライニングトルク・荷重依存性を検証する単体テストです。

## 2. 実行上の役割

- 学習系タイヤモデルが使えない場合のフォールバック候補として、Magic Formula 実装の基礎妥当性を保証します。
- 広い入力範囲で有限値を返すことも確認し、数値健全性を担保します。

## 3. 依存関係

- `fsae/physics/tire/magic_formula.hpp`
  - `MagicFormulaParams`
  - `compute_magic_formula`
  - `compute_mf_Fx0`
  - `compute_mf_Fy0`
  - `compute_mf_Gxa`
  - `compute_mf_Gyk`
- `gtest/gtest.h`

## 4. 入力 / 出力

- 入力
  - Magic Formula パラメータ
  - スリップ率 `kappa`
  - スリップ角 `alpha`
  - 垂直荷重 `Fz`
  - キャンバー `gamma`
- 出力
  - `Fx`, `Fy`, `Mz`
  - 正規化低減係数 `Gxa`, `Gyk`

## 5. 関数・メソッド・クラスの網羅的な説明

### `class MagicFormulaTest : public ::testing::Test`

- Hoosier 18x6-10 R20 相当を意識した代表パラメータを設定するフィクスチャです。

#### `void SetUp() override`

- 純横力、純縦力、複合スリップ係数、アライニングトルク係数をまとめて初期化します。

#### メンバ

- `MagicFormulaParams params`

### `TEST_F(MagicFormulaTest, ZeroSlipProducesNearZeroForce)`

- ゼロスリップ時に `Fx`, `Fy` がほぼ 0 になることを確認します。

### `TEST_F(MagicFormulaTest, PureLateralForceSCurve)`

- スリップ角掃引で横力が S 字型かつ正負反対符号になることを確認します。

### `TEST_F(MagicFormulaTest, PureLateralForcePeakValue)`

- ピーク横力が物理的に妥当な範囲に入ることを確認します。

### `TEST_F(MagicFormulaTest, PureLongitudinalForcePeakValue)`

- ピーク縦力が妥当レンジに収まることを確認します。

### `TEST_F(MagicFormulaTest, CombinedSlipGxaRange)`

- 縦力側の複合スリップ低減係数 `Gxa` が 0〜1 付近に収まり、角度増加で低下することを確認します。

### `TEST_F(MagicFormulaTest, CombinedSlipGykRange)`

- 横力側低減係数 `Gyk` が 0〜1 付近に収まることを確認します。

### `TEST_F(MagicFormulaTest, SelfAligningTorqueSmallAtZeroSlip)`

- ゼロスリップ時のセルフアライニングトルクが残留成分を除き小さいことを確認します。

### `TEST_F(MagicFormulaTest, LoadDependence)`

- 荷重増加に応じて横力絶対値も増えることを確認します。

### `TEST_F(MagicFormulaTest, AllOutputsFinite)`

- 荷重・スリップ率・スリップ角・キャンバーの広域組合せで NaN / Inf が出ないことを確認します。

### `TEST_F(MagicFormulaTest, TopLevelCombinedReduction)`

- 複合スリップ時の力が、純スリップ時の力を上回らないことを確認します。
